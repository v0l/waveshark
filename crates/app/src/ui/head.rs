//! The top bar.
//!
//! This one stays on `App` rather than becoming a pane over its own state.
//! It is not a view of anything: it is where the receiver itself is set, so
//! the device, the span, the dial and the transport are the application's own
//! fields, and a struct borrowing them would borrow most of the app.
//!
//! The row is a strip of cells: a legend, and under it the one control or
//! readout the legend names. Every cell starts at the same y and the row has
//! a fixed height, so nothing on the bar moves when a device name gets longer
//! or an update appears. What the operator sets is on the left, starting at
//! the dial; what the receiver reports and the windows it opens are pinned to
//! the right edge, where they stay put as the window is resized.

use super::*;

/// Height of the cells, and so of the bar.
///
/// Fixed rather than grown from the content: the tallest cell is the receiver,
/// whose transport sits under its device box, and letting the row size itself
/// meant the whole interface shifted down when a cell appeared.
const CELL_H: f32 = 74.0;

/// Gap either side of a divider. The dividers are the only separation in the
/// row, so they carry the rhythm and every gap is this one.
const GAP: f32 = 16.0;

/// Width of the device box, and so of the transport strip beneath it.
const DEVICE_W: f32 = 190.0;

/// Side of the icon buttons in the segmented strips. Smaller than the
/// toolbar's, because two of them stack under a combo box inside one cell.
const ICON: f32 = 22.0;

/// The rest of the cell widths. Fixed for the same reason the height is: a
/// bar whose columns move as a band name or a version number changes length
/// is a bar nobody can find anything on twice.
const SPAN_W: f32 = 104.0;
/// Side of a view tab. Larger than the transport's icons because these are
/// the only route to a view by pointer and they carry a glyph that has to be
/// told from nine others, not a play triangle.
const TAB: f32 = 34.0;
/// Ten tabs, the gap between the two groups, and the well's margin.
const VIEW_W: f32 = TAB * 10.0 + 2.0 * 9.0 + 5.0 + 8.0;
/// How a view's shortcut is written in its hover text.
#[cfg(target_os = "macos")]
const TAB_MOD: &str = "\u{2318}";
#[cfg(not(target_os = "macos"))]
const TAB_MOD: &str = "Ctrl+";

const SPEED_W: f32 = 100.0;
const PANELS_W: f32 = 88.0;
const UPDATE_W: f32 = 72.0;

impl App {
    /// The readout and the controls that set it.
    pub(super) fn head(&mut self, ui: &mut egui::Ui) {
        Panel::top("head")
            .frame(
                egui::Frame::NONE.fill(theme::PANEL).inner_margin(egui::Margin::symmetric(14, 8)),
            )
            .show(ui, |ui| {
                // Top-aligned: the row is as tall as the receiver cell, and
                // centring left every shorter cell's legend on a line of its
                // own.
                ui.horizontal_top(|ui| {
                    ui.set_height(CELL_H);
                    // The band under the dial, not beside it. It is not a
                    // control and it is not a subject of its own: it is what
                    // the number above it means, and a cell of its own put a
                    // legend and a divider around a caption.
                    ui.vertical(|ui| {
                        let out = self.dial.show_tunable(ui, self.center, 34.0, self.tunable);
                        if out.changed {
                            self.retune(out.hz);
                        }
                        ui.add_space(3.0);
                        ui.horizontal(|ui| {
                            ui.add_space(2.0);
                            // A caption on the dial, in the caption's own
                            // colour. Cyan is what the radio heard, and a
                            // band plan is not heard: it is looked up from
                            // the number above it.
                            theme::Line::new().legend(bands::name_at(self.center)).show(ui);
                        });
                    });

                    self.rule(ui);
                    self.receiver_cell(ui);
                    self.rule(ui);
                    self.span_cell(ui);
                    self.rule(ui);
                    self.view_cell(ui);

                    // Everything left over goes here, which is what puts the
                    // tail against the right edge however wide the window is.
                    // A right-to-left layout allocated at the cursor sizes
                    // itself to its content instead, which is how the setup
                    // button ended up sitting in the middle of the bar.
                    // A right-to-left layout draws over whatever is to its
                    // left when the row has run out of width, and what it
                    // drew over was the view strip. Claiming a floor keeps
                    // the bar wider than the window instead, which clips at
                    // the edge rather than stacking two cells on one another.
                    let rest = ui.available_width().max(PANELS_W + 12.0);
                    // The speed trace is a readout and the strip beside it
                    // is a control, so on a narrow window the readout is the
                    // one that goes.
                    let room = rest > SPEED_W + PANELS_W + GAP * 2.0;
                    ui.allocate_ui_with_layout(
                        Vec2::new(rest, CELL_H),
                        egui::Layout::right_to_left(egui::Align::Min),
                        |ui| {
                            self.panels_cell(ui);
                            if room {
                                self.rule(ui);
                                self.speed_cell(ui);
                                self.update_badge(ui);
                            }
                        },
                    );
                });
            });
    }

    /// The device, and the transport that acts on it.
    ///
    /// One cell because they are one subject: the buttons start, stop,
    /// configure and record whatever the box above them names, and split
    /// across the bar they read as four unrelated switches.
    fn receiver_cell(&mut self, ui: &mut egui::Ui) {
        let mut pick = None;
        let mut rescan = false;
        let mut forget = None;
        let mut add_remote = false;
        let mut open_radio = false;
        let mut connect = false;
        let mut stop = false;
        let mut capture: Option<bool> = None;
        cell(ui, "receiver", DEVICE_W, |ui| {
            let cur =
                self.device.as_ref().map(|d| d.label.clone()).unwrap_or_else(|| "none".into());
            egui::ComboBox::from_id_salt("device").selected_text(cur).width(DEVICE_W).show_ui(
                ui,
                |ui| {
                    for d in &self.devices {
                        let on = self.device.as_ref() == Some(d);
                        // A remote radio was created here rather than plugged
                        // in, so it is dropped here too: nothing else in the
                        // interface knows it exists.
                        match &d.addr {
                            Some(addr) => {
                                ui.horizontal(|ui| {
                                    if ui.selectable_label(on, &d.label).clicked() {
                                        pick = Some(d.clone());
                                    }
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            if ui.small_button("×").clicked() {
                                                forget = Some(addr.clone());
                                            }
                                        },
                                    );
                                });
                            }
                            None => {
                                if ui.selectable_label(on, &d.label).clicked() {
                                    pick = Some(d.clone());
                                }
                            }
                        }
                    }
                    ui.separator();
                    if ui.selectable_label(false, "Rescan").clicked() {
                        rescan = true;
                    }
                    if ui.selectable_label(false, "Add remote…").clicked() {
                        add_remote = true;
                    }
                },
            );

            ui.add_space(4.0);
            // One segmented control rather than four loose icons. Four
            // unbordered glyphs floating under a combo box did not read as
            // controls at all, and the recording switch beside them read as
            // decoration.
            let on = self.radio.is_some();
            let capturing = self
                .radio
                .as_ref()
                .is_some_and(|r| r.status.capture_on.load(std::sync::atomic::Ordering::Relaxed));
            segment(ui, DEVICE_W, |ui| {
                use crate::icons::{icon_button_sized, Icon};
                let t = crate::i18n::t;
                // Stopping releases the USB claim, which is the only way to
                // hand the radio to another program without quitting.
                connect =
                    icon_button_sized(ui, Icon::Play, t("ui.start"), !on, false, ICON).clicked();
                stop = icon_button_sized(ui, Icon::Stop, t("ui.stop"), on, false, ICON).clicked();
                // Not gain alone: the pane behind it also holds the radio's
                // switches, its antenna and channel choices, and the crystal
                // correction.
                open_radio =
                    icon_button_sized(ui, Icon::Sliders, t("ui.settings"), on, false, ICON)
                        .clicked();
                // Beside the transport, because that is what it is: the span
                // is running and this writes it down. A signal nothing
                // decodes is worth capturing while it is still transmitting,
                // and anything behind a modal is too slow for that.
                let tip = if capturing { t("ui.capture_stop") } else { t("ui.capture") };
                if icon_button_sized(ui, Icon::Capture, tip, on, capturing, ICON).clicked() {
                    capture = Some(!capturing);
                }
            });
        });

        if add_remote {
            self.remote = Some(RemoteEdit::default());
        }
        if let Some(addr) = forget {
            crate::devices::remove_stream(&addr);
            let c = ui.ctx().clone();
            self.rescan(&c);
        }
        if rescan {
            let c = ui.ctx().clone();
            self.rescan(&c);
        }
        if let Some(d) = pick {
            let c = ui.ctx().clone();
            self.select_device(&c, d);
        }
        if connect {
            let c = ui.ctx().clone();
            self.connect(&c);
        }
        if stop {
            self.stop();
        }
        if open_radio {
            self.open = Some(Settings::Radio);
        }
        if let Some(want) = capture {
            self.set_capture(want);
        }
    }

    /// How much of the band is being received, and at what decimation.
    fn span_cell(&mut self, ui: &mut egui::Ui) {
        let mut pick = None;
        cell(ui, "span", SPAN_W, |ui| {
            let cur = self
                .spans
                .iter()
                .find(|s| (self.rate - s.effective()).abs() < 1.0)
                .map(|s| s.label.clone())
                .unwrap_or_else(|| "custom".into());
            egui::ComboBox::from_id_salt("span").selected_text(cur).width(SPAN_W).show_ui(
                ui,
                |ui| {
                    for sp in &self.spans {
                        let on = (self.rate - sp.effective()).abs() < 1.0 && self.zoom == sp.zoom;
                        let text = if sp.zoom > 1 {
                            format!("{}  /{}", sp.label, sp.zoom)
                        } else {
                            sp.label.clone()
                        };
                        if ui.selectable_label(on, text).clicked() && !on {
                            pick = Some(sp.clone());
                        }
                    }
                },
            );
            // Decimation is not visible in the span label, and a receiver
            // running at a quarter rate behaves differently enough to say so.
            if self.zoom > 1 {
                ui.add_space(4.0);
                ui.label(value(format!("/{} zoom", self.zoom)).color(theme::LEGEND).size(11.0));
            }
        });
        if let Some(sp) = pick {
            // Rate first: the radio rebuilds everything on a rate change, and
            // a zoom sent before it would be applied to a chain about to be
            // replaced.
            self.send(Cmd::Rate(Sps(sp.rate as u64)));
            self.send(Cmd::Zoom(sp.zoom));
            self.rate = sp.effective();
            self.zoom = sp.zoom;
            self.reset_waterfall();
            self.retune_listener();
        }
    }

    /// Which window fills the middle of the screen.
    ///
    /// A strip of tabs rather than a dropdown. Eleven views behind a combo box
    /// cost two clicks and a read of a menu every time, which is most of a
    /// second to look at the map and another to come back; as tabs it is one
    /// click, the choice is visible without opening anything, and a dot on a
    /// tab says which views are holding traffic.
    ///
    /// One row rather than two of five. Two rows kept the cell as narrow as
    /// the dropdown was, but only by drawing the glyphs at the size of the
    /// transport buttons, which is too small to tell a dish from a handset in
    /// passing; a row of full-size tabs spends the width the bar
    /// has spare and puts the tabs in the order of their shortcuts. The gap
    /// in the middle is the join between what the receiver is doing and who
    /// is out there.
    fn view_cell(&mut self, ui: &mut egui::Ui) {
        let mut pick = None;
        ui.allocate_ui_with_layout(
            Vec2::new(VIEW_W, CELL_H),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
                ui.set_min_size(Vec2::new(VIEW_W, CELL_H));
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                // The legend doubles as the readout: with no room to label
                // ten tabs, the one that is on has to be spelled where the
                // eye already goes for the cell's name.
                theme::Line::new()
                    .legend("view")
                    .gap(6.0)
                    .legend(self.view.label())
                    .tint(theme::READOUT)
                    .show(ui);
                ui.add_space(3.0);
                egui::Frame::NONE
                    .fill(theme::WELL)
                    .stroke(Stroke::new(1.0, theme::ETCH))
                    .inner_margin(egui::Margin::symmetric(4, 3))
                    .corner_radius(2)
                    .show(ui, |ui| {
                        ui.spacing_mut().item_spacing = Vec2::new(2.0, 2.0);
                        ui.horizontal(|ui| {
                            let mut n = 0;
                            for (i, row) in self.tabs().into_iter().enumerate() {
                                if i > 0 {
                                    ui.add_space(5.0);
                                }
                                for v in row.iter().copied() {
                                    let key = tab_digit(n);
                                    n += 1;
                                    let tip = match key {
                                        Some((_, d)) => format!(
                                            "{}  ({}{})\n{}",
                                            v.label(),
                                            TAB_MOD,
                                            d,
                                            v.about()
                                        ),
                                        None => format!("{}\n{}", v.label(), v.about()),
                                    };
                                    let hit = crate::icons::icon_tab(
                                        ui,
                                        v.icon(),
                                        &tip,
                                        self.view == v,
                                        self.view_live(v),
                                        TAB,
                                    );
                                    if hit.clicked() {
                                        pick = Some(v);
                                    }
                                }
                            }
                        });
                    });
            },
        );
        if let Some(v) = pick {
            self.set_view(v);
        }
    }

    /// The windows that open over the view: the packet log, the cached
    /// datasets and setup.
    ///
    /// Together at the right end because they are the same kind of thing, a
    /// panel that appears rather than a property of the receiver, and none of
    /// them belongs in the reading half of the bar.
    fn panels_cell(&mut self, ui: &mut egui::Ui) {
        let mut log = false;
        let mut data = false;
        let mut setup = false;
        cell(ui, "panels", PANELS_W, |ui| {
            segment(ui, PANELS_W, |ui| {
                use crate::icons::{icon_button_sized, Icon};
                // Only the switch that opens the log. What decodes and what
                // runs where are questions about the packets, so they are
                // asked in the window that shows them rather than up here.
                log = icon_button_sized(
                    ui,
                    Icon::Log,
                    crate::i18n::t("ui.log"),
                    true,
                    self.log.open,
                    ICON,
                )
                .clicked();
                data = icon_button_sized(
                    ui,
                    Icon::Data,
                    crate::i18n::t("ui.data"),
                    true,
                    self.open == Some(Settings::Data),
                    ICON,
                )
                .clicked();
                setup = icon_button_sized(
                    ui,
                    Icon::Setup,
                    crate::i18n::t("ui.setup"),
                    true,
                    self.open == Some(Settings::App),
                    ICON,
                )
                .clicked();
            });
        });
        if log {
            self.log.open = !self.log.open;
        }
        if data {
            // A second press closes it, like the log: this is a window, not
            // a place to be taken to.
            self.open = match self.open {
                Some(Settings::Data) => None,
                _ => Some(Settings::Data),
            };
        }
        if setup {
            self.open = Some(Settings::App);
        }
    }

    /// What the receiver is managing, as a trace and a number.
    fn speed_cell(&mut self, ui: &mut egui::Ui) {
        cell(ui, "speed", SPEED_W, |ui| {
            let now = self.speed_now();
            self.status_lamp(ui);
            ui.add_space(2.0);
            ui.label(match now {
                Some(x) => value(format!("{x:.1}x")).color(theme::LEGEND).size(11.0),
                None => value("stopped").color(theme::LEGEND).size(11.0),
            });
        });
    }

    /// The one thing the release check has to say up here: a newer version
    /// exists.
    ///
    /// Drawn only then. A row that reads "up to date" every day teaches an
    /// operator to stop looking at it, and the rest of the answer, which
    /// release, which archive, how big, is in Setup where there is room for
    /// it. Pressing this opens that pane rather than a browser: what to do
    /// about an update is a decision, not a click.
    fn update_badge(&mut self, ui: &mut egui::Ui) {
        let mut open = false;
        let state = crate::update::state();
        // The check runs off its own thread and finishes seconds after the
        // window opens, so a receiver sitting stopped would otherwise not
        // repaint until the pointer moved.
        if matches!(state, crate::update::State::Checking) {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(500));
        }
        let crate::update::State::Newer(r) = state else {
            return;
        };
        self.rule(ui);
        cell(ui, "update", UPDATE_W, |ui| {
            let hover = match &r.asset {
                Some(a) => format!(
                    "{} is out; this is {}. Open Setup for the release, or {}.",
                    r.version,
                    crate::update::running(),
                    a.name
                ),
                None => format!(
                    "{} is out; this is {}, and that release carries no {} archive.",
                    r.version,
                    crate::update::running(),
                    crate::update::platform()
                ),
            };
            let text = value(format!("v{}", r.version)).color(theme::OK).size(13.0);
            open = ui.button(text).on_hover_text(hover).clicked();
        });
        if open {
            self.open = Some(Settings::App);
        }
    }

    /// Real time multiples the graph is currently running at, or `None` when
    /// there is no radio or nothing has been timed yet.
    fn speed_now(&self) -> Option<f32> {
        let r = self.radio.as_ref()?;
        if !r.status.running.load(std::sync::atomic::Ordering::Relaxed) {
            return None;
        }
        r.status.speed_history().last().copied()
    }

    /// A divider with its gap either side, so a caller cannot get the rhythm
    /// wrong by leaving one of the three out.
    fn rule(&self, ui: &mut egui::Ui) {
        ui.add_space(GAP);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(1.0, CELL_H - 6.0), Sense::hover());
        ui.painter().line_segment(
            [Pos2::new(rect.center().x, rect.top()), Pos2::new(rect.center().x, rect.bottom())],
            Stroke::new(1.0, theme::ETCH),
        );
        ui.add_space(GAP);
    }

    /// How fast the graph is running against real time. Drawn by
    /// [`widgets::speed_trace`], which the dashboard shows larger.
    fn status_lamp(&self, ui: &mut egui::Ui) {
        use std::sync::atomic::Ordering;
        let (running, dropped, hist) = match &self.radio {
            Some(r) => (
                r.status.running.load(Ordering::Relaxed),
                r.status.dropped.load(Ordering::Relaxed),
                r.status.speed_history(),
            ),
            None => (false, 0, Vec::new()),
        };
        widgets::speed_trace(ui, Vec2::new(SPEED_W, 20.0), running, dropped, &hist);
    }
}

/// One cell of the bar: a legend, and under it whatever the legend names.
///
/// Laid out top-down at a fixed height so that every legend on the row shares
/// a baseline and every control under one starts at the same y. Cells used to
/// be written out at each site, which is why no two of them lined up.
fn cell(ui: &mut egui::Ui, name: &str, width: f32, content: impl FnOnce(&mut egui::Ui)) {
    // The width is given rather than grown from the content. A cell that
    // sizes itself is handed the whole remaining row inside a right-to-left
    // layout and draws its contents at the far end of it, which is how the
    // right-hand cells ended up in the middle of the bar.
    ui.allocate_ui_with_layout(
        Vec2::new(width, CELL_H),
        egui::Layout::top_down(egui::Align::Min),
        |ui| {
            ui.set_min_size(Vec2::new(width, CELL_H));
            ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
            ui.label(legend(name));
            ui.add_space(3.0);
            content(ui);
        },
    );
}

/// A strip of icon buttons that reads as one control rather than as loose
/// glyphs: a well, an engraved border, and no gap between the buttons.
fn segment(ui: &mut egui::Ui, width: f32, content: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::NONE
        .fill(theme::WELL)
        .stroke(Stroke::new(1.0, theme::ETCH))
        .inner_margin(egui::Margin::symmetric(3, 2))
        .corner_radius(2)
        .show(ui, |ui| {
            if width > 0.0 {
                ui.set_width(width - 8.0);
            }
            ui.spacing_mut().item_spacing.x = 2.0;
            ui.horizontal(|ui| content(ui));
        });
}
