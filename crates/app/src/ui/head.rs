//! The top bar.
//!
//! This one stays on `App` rather than becoming a pane over its own state.
//! It is not a view of anything: it is where the receiver itself is set, so
//! the device, the span, the dial and the transport are the application's own
//! fields, and a struct borrowing them would borrow most of the app.

use super::*;

/// Full scale of the speed trace, in octaves either side of real time: the
/// top of the well is 8x and the bottom is an eighth.
const SPEED_DECADES: f32 = 3.0;

/// Below this the margin is thin enough to be worth saying so before a block
/// is actually late.
const SPEED_TIGHT: f32 = 1.5;

impl App {
    /// The readout and the controls that set it.
    pub(super) fn head(&mut self, ui: &mut egui::Ui) {
        Panel::top("head")
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(14, 10)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let out = self.dial.show(ui, self.center, 34.0);
                    if out.changed {
                        self.retune(out.hz);
                    }

                    ui.add_space(16.0);
                    ui.vertical(|ui| {
                        ui.add_space(2.0);
                        ui.label(legend("band"));
                        ui.label(
                            value(bands::name_at(self.center))
                                .color(theme::TRACE)
                                .size(14.0),
                        );
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            ui.label(legend("radio"));
                            // Beside the device it describes. Off in the
                            // corner it was a readout of something, and which
                            // something was anyone's guess.
                            self.status_lamp(ui);
                        });
                        let cur = self
                            .device
                            .as_ref()
                            .map(|d| d.label.clone())
                            .unwrap_or_else(|| "none".into());
                        let mut pick = None;
                        let mut rescan = false;
                        let mut forget = None;
                        let mut add_remote = false;
                        egui::ComboBox::from_id_salt("device")
                            .selected_text(cur)
                            .width(190.0)
                            .show_ui(ui, |ui| {
                                for d in &self.devices {
                                    let on = self.device.as_ref() == Some(d);
                                    // A remote radio was created here rather
                                    // than plugged in, so it is dropped here
                                    // too: nothing else in the interface knows
                                    // it exists.
                                    match &d.addr {
                                        Some(addr) => {
                                            ui.horizontal(|ui| {
                                                if ui.selectable_label(on, &d.label).clicked() {
                                                    pick = Some(d.clone());
                                                }
                                                ui.with_layout(
                                                    egui::Layout::right_to_left(
                                                        egui::Align::Center,
                                                    ),
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

                        ui.add_space(4.0);
                        ui.horizontal(|ui| {
                            // Stopping releases the USB claim, which is the
                            // only way to hand the radio to another program
                            // without quitting.
                            use crate::icons::{icon_button, Icon};
                            let on = self.radio.is_some();
                            let t = crate::i18n::t;
                            if icon_button(ui, Icon::Play, t("ui.start"), !on, false).clicked() {
                                let c = ui.ctx().clone();
                                self.connect(&c);
                            }
                            if icon_button(ui, Icon::Stop, t("ui.stop"), on, false).clicked() {
                                self.stop();
                            }
                            // Not gain alone: the pane behind it also holds
                            // the radio's switches, its antenna and channel
                            // choices, and the crystal correction.
                            if icon_button(ui, Icon::Sliders, t("ui.settings"), on, false)
                                .clicked()
                            {
                                self.open = Some(Settings::Radio);
                            }
                            // Beside the transport, because that is what it
                            // is: the span is running and this writes it
                            // down. A signal nothing decodes is worth
                            // capturing while it is still transmitting, and
                            // anything behind a modal is too slow for that.
                            let capturing = self
                                .radio
                                .as_ref()
                                .is_some_and(|r| r.status.capture_on.load(std::sync::atomic::Ordering::Relaxed));
                            let tip = if capturing { t("ui.capture_stop") } else { t("ui.capture") };
                            if icon_button(ui, Icon::Capture, tip, on, capturing).clicked() {
                                self.set_capture(!capturing);
                            }
                        });
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("bandwidth"));
                        let cur = self
                            .spans
                            .iter()
                            .find(|s| (self.rate - s.effective()).abs() < 1.0)
                            .map(|s| s.label.clone())
                            .unwrap_or_else(|| "custom".into());
                        let mut pick = None;
                        egui::ComboBox::from_id_salt("span")
                            .selected_text(cur)
                            .width(96.0)
                            .show_ui(ui, |ui| {
                                for sp in &self.spans {
                                    let on = (self.rate - sp.effective()).abs() < 1.0
                                        && self.zoom == sp.zoom;
                                    let text = if sp.zoom > 1 {
                                        format!("{}  /{}", sp.label, sp.zoom)
                                    } else {
                                        sp.label.clone()
                                    };
                                    if ui.selectable_label(on, text).clicked() && !on {
                                        pick = Some(sp.clone());
                                    }
                                }
                            });
                        if let Some(sp) = pick {
                            // Rate first: the radio rebuilds everything on a
                            // rate change, and a zoom sent before it would be
                            // applied to a chain about to be replaced.
                            self.send(Cmd::Rate(Sps(sp.rate as u64)));
                            self.send(Cmd::Zoom(sp.zoom));
                            self.rate = sp.effective();
                            self.zoom = sp.zoom;
                            self.reset_waterfall();
                            self.retune_listener();
                        }
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("view"));
                        let mut v = self.view;
                        egui::ComboBox::from_id_salt("view")
                            .selected_text(v.label())
                            .width(120.0)
                            .show_ui(ui, |ui| {
                                for opt in [View::Spectrum, View::Chain, View::Map, View::Calls] {
                                    ui.selectable_value(&mut v, opt, opt.label());
                                }
                            });
                        self.view = v;
                    });

                    ui.add_space(18.0);
                    self.divider(ui);
                    ui.add_space(18.0);

                    ui.vertical(|ui| {
                        ui.label(legend("packets"));
                        ui.horizontal(|ui| {
                            // Only the switch that opens the log. What decodes
                            // and what runs where are questions about the
                            // packets, so they are asked in the window that
                            // shows them rather than up here.
                            if crate::icons::icon_button(
                                ui,
                                crate::icons::Icon::Log,
                                crate::i18n::t("ui.log"),
                                true,
                                self.log.open,
                            )
                            .clicked()
                            {
                                self.log.open = !self.log.open;
                            }
                        });
                    });

                    // Pinned to the far end, and built like every other group
                    // on this row: a legend with its control under it. Floating
                    // loose against the right edge, it read as a lamp or a
                    // dismiss button, because nothing said what it belonged to.
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                        ui.vertical(|ui| {
                            ui.label(legend("setup"));
                            ui.horizontal(|ui| {
                                let open = crate::icons::icon_button(
                                    ui,
                                    crate::icons::Icon::Setup,
                                    crate::i18n::t("ui.setup"),
                                    true,
                                    self.open == Some(Settings::App),
                                );
                                if open.clicked() {
                                    self.open = Some(Settings::App);
                                }
                            });
                        });
                        ui.add_space(18.0);
                        self.divider(ui);
                        ui.add_space(18.0);
                    });
                });

            if let Some(e) = &self.err {
                    ui.add_space(4.0);
                    ui.label(
                        egui::RichText::new(e)
                            .color(theme::FAULT)
                            .font(FontId::proportional(12.0)),
                    );
                }
            });
    }

    fn divider(&self, ui: &mut egui::Ui) {
        let h = 40.0;
        let (rect, _) = ui.allocate_exact_size(Vec2::new(1.0, h), Sense::hover());
        ui.painter().line_segment(
            [
                Pos2::new(rect.center().x, rect.top()),
                Pos2::new(rect.center().x, rect.bottom()),
            ],
            Stroke::new(1.0, theme::ETCH),
        );
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

        let (rect, resp) = ui.allocate_exact_size(Vec2::new(84.0, 18.0), Sense::hover());
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
