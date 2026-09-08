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

/// Full scale of the speed trace, in octaves either side of real time: the
/// top of the well is 8x and the bottom is an eighth.
const SPEED_DECADES: f32 = 3.0;

/// Below this the margin is thin enough to be worth saying so before a block
/// is actually late.
const SPEED_TIGHT: f32 = 1.5;

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
const BAND_W: f32 = 96.0;
const SPAN_W: f32 = 104.0;
const VIEW_W: f32 = 128.0;
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
                    let out = self.dial.show_tunable(ui, self.center, 34.0, self.tunable);
                    if out.changed {
                        self.retune(out.hz);
                    }

                    ui.add_space(GAP);
                    cell(ui, "band", BAND_W, |ui| {
                        ui.label(value(bands::name_at(self.center)).color(theme::TRACE).size(15.0));
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
                    let rest = ui.available_width();
                    ui.allocate_ui_with_layout(
                        Vec2::new(rest, CELL_H),
                        egui::Layout::right_to_left(egui::Align::Min),
                        |ui| {
                            self.panels_cell(ui);
                            self.rule(ui);
                            self.speed_cell(ui);
                            self.update_badge(ui);
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
    fn view_cell(&mut self, ui: &mut egui::Ui) {
        cell(ui, "view", VIEW_W, |ui| {
            let mut v = self.view;
            egui::ComboBox::from_id_salt("view").selected_text(v.label()).width(VIEW_W).show_ui(
                ui,
                |ui| {
                    for opt in [
                        View::Spectrum,
                        View::Chain,
                        View::Map,
                        View::Calls,
                        View::Messages,
                        View::Video,
                        View::Links,
                        View::Devices,
                        View::Satellites,
                        View::Keys,
                    ] {
                        ui.selectable_value(&mut v, opt, opt.label());
                    }
                },
            );
            self.view = v;
        });
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

    /// How fast the graph is running against real time, drawn as a trace over
    /// a line at 1x.
    ///
    /// A lamp lived here and said too little: green meant "nothing has been
    /// dropped yet", which is the same colour whether the host has ten times
    /// the headroom it needs or is a hair from falling over. What an operator
    /// about to add a channel wants is the margin, and the margin is only
    /// readable against real time, so the trace is drawn against a 1x rule.
    /// Touching that rule is the warning; crossing it is the fault, and the
    /// dropped count that used to be the whole reading is behind the hover.
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
        let now = hist.last().copied().unwrap_or(0.0);
        let worst = hist.iter().copied().fold(f32::INFINITY, f32::min);
        let col = if !running || dropped > 0 || worst < 1.0 {
            theme::FAULT
        } else if worst < SPEED_TIGHT {
            theme::READOUT
        } else {
            theme::OK
        };

        let (rect, resp) = ui.allocate_exact_size(Vec2::new(SPEED_W, 20.0), Sense::hover());
        let p = ui.painter();
        p.rect_filled(rect, 1.0, theme::WELL);
        p.rect_stroke(rect, 1.0, Stroke::new(1.0, theme::ETCH), egui::StrokeKind::Inside);

        // Ratios, so 2x above the line has to look like half speed below it;
        // on a linear axis everything slow is squashed into the bottom pixel.
        let plot = rect.shrink(2.0);
        let y = |v: f32| {
            let t = (v.max(0.03).log2() / SPEED_DECADES).clamp(-1.0, 1.0);
            plot.center().y - t * plot.height() / 2.0
        };
        let one = y(1.0);
        for x in (0..plot.width() as i32).step_by(4) {
            let x = plot.left() + x as f32;
            p.line_segment(
                [Pos2::new(x, one), Pos2::new((x + 2.0).min(plot.right()), one)],
                Stroke::new(1.0, theme::LEGEND.gamma_multiply(0.7)),
            );
        }

        if hist.len() > 1 {
            let step = plot.width() / (hist.len() - 1) as f32;
            let pts: Vec<Pos2> = hist
                .iter()
                .enumerate()
                .map(|(i, v)| Pos2::new(plot.left() + i as f32 * step, y(*v)))
                .collect();
            p.add(egui::Shape::line(pts, Stroke::new(1.0, col)));
        }

        resp.on_hover_text(if !running {
            "Stopped. The device is free for another program.".to_string()
        } else if hist.is_empty() {
            "Receiving. No block has been timed yet.".to_string()
        } else if dropped == 0 {
            format!(
                "Running at {now:.1}x real time, worst {worst:.1}x of the last {} blocks. \
                 No samples dropped.",
                hist.len()
            )
        } else {
            format!(
                "Running at {now:.1}x real time, worst {worst:.1}x, and {} samples were dropped: \
                 the host is not keeping up with this span.",
                thousands(dropped)
            )
        });
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
