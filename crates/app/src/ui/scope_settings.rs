//! The spectrum, waterfall and scale panels, over the scope's own settings.
//!
//! These belong to the scope rather than to the application: they set what it
//! draws and how, and the panel is the same state the pane reads.

use super::state::ScopeState;
use super::*;

/// What the panels want done that they cannot do themselves.
pub(super) enum Action {
    /// The span or the bin count changed, so old rows no longer line up.
    ResetWaterfall,
}

/// The scope's settings, over the state they change.
pub(super) struct ScopeSettings<'a> {
    pub st: &'a mut ScopeState,
    /// Removing the direct-conversion centre spur, which is a property of the
    /// receiver rather than of the drawing but is set beside it.
    pub dc_block: &'a mut bool,
    pub rate: f64,
    pub cmds: &'a mut Vec<Cmd>,
    pub acts: Vec<Action>,
}

impl ScopeSettings<'_> {
    /// The spectrum panel: what the transform is doing, and the scale it is
    /// drawn against.
    pub(super) fn spectrum(&mut self, ui: &mut egui::Ui) {
        row(ui, "FFT bins", |ui| {
            let mut n = self.st.fft_size;
            egui::ComboBox::from_id_salt("fft")
                .selected_text(n.to_string())
                .width(120.0)
                .show_ui(ui, |ui| {
                    for v in FFTS {
                        ui.selectable_value(&mut n, v, v.to_string());
                    }
                });
            if n != self.st.fft_size {
                self.st.fft_size = n;
                // The same value the session saves and the radio starts with,
                // so a chosen FFT size survives a restart rather than only
                // living in the running spectrum.
                self.st.fft = n;
                self.cmds.push(Cmd::Fft(n));
                self.acts.push(Action::ResetWaterfall);
            }
        });
        ui.label(
            egui::RichText::new(bin_hint(self.rate, self.st.fft_size))
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Refresh", |ui| {
            let mut v = self.st.refresh;
            egui::ComboBox::from_id_salt("fps")
                .selected_text(format!("{} fps", v as i32))
                .width(120.0)
                .show_ui(ui, |ui| {
                    for (n, f) in REFRESH {
                        ui.selectable_value(&mut v, f, format!("{n} fps"));
                    }
                });
            if (v - self.st.refresh).abs() > 0.01 {
                self.st.refresh = v;
                self.cmds.push(Cmd::Refresh(v));
            }
        });
        ui.add_space(8.0);

        row(ui, "Averaging", |ui| {
            if ui
                .add(egui::Slider::new(&mut self.st.smoothing, 0.02..=1.0).show_value(false))
                .changed()
            {
                self.cmds.push(Cmd::Smoothing(self.st.smoothing));
            }
            ui.label(value(if self.st.smoothing > 0.95 {
                "off".to_string()
            } else {
                format!("{:.0}%", (1.0 - self.st.smoothing) * 100.0)
            }));
        });
        ui.add_space(8.0);

        row(ui, "Centre spur", |ui| {
            if ui.checkbox(&mut *self.dc_block, "Remove").changed() {
                self.cmds.push(Cmd::DcBlock(*self.dc_block));
            }
            ui.label(
                egui::RichText::new("LO leakage at the tuned frequency")
                    .color(theme::LEGEND)
                    .size(10.0),
            );
        });
        ui.add_space(8.0);
        self.scale(ui);
    }

    /// The waterfall panel: how fast it scrolls, and how much it keeps.
    pub(super) fn waterfall(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scroll rate", |ui| {
            let mut v = self.st.rows_per_sec;
            egui::ComboBox::from_id_salt("rows")
                .selected_text(format!("{} rows/s", v as i32))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for (n, f) in SPEEDS {
                        ui.selectable_value(&mut v, f, format!("{n} rows/s"));
                    }
                });
            self.st.rows_per_sec = v;
        });
        ui.add_space(8.0);

        row(ui, "History", |ui| {
            let mut n = self.st.wf_rows;
            egui::ComboBox::from_id_salt("hist")
                .selected_text(format!("{n} rows"))
                .width(130.0)
                .show_ui(ui, |ui| {
                    for v in [256usize, 512, 1024, 2048] {
                        ui.selectable_value(&mut n, v, format!("{v} rows"));
                    }
                });
            if n != self.st.wf_rows {
                self.st.wf_rows = n;
                self.st.wf.set_height(n);
            }
        });
        ui.label(
            egui::RichText::new(format!(
                "{:.0} s of history at {:.0} rows/s",
                self.st.wf.height() as f32 / self.st.rows_per_sec,
                self.st.rows_per_sec
            ))
            .small()
            .color(theme::LEGEND),
        );
        ui.add_space(8.0);

        row(ui, "Contrast", |ui| {
            ui.add(egui::Slider::new(&mut self.st.wf_top_offset, 0.0..=20.0).show_value(false));
            ui.label(value(format!("{:.0} dB", self.st.wf_top_offset)));
        });
        ui.label(
            egui::RichText::new("How far below the trace ceiling the hottest colour sits.")
                .small()
                .color(theme::LEGEND),
        );
        ui.add_space(8.0);
        self.scale(ui);
    }

    /// The decibel scale both of them are drawn against.
    fn scale(&mut self, ui: &mut egui::Ui) {
        row(ui, "Scale", |ui| {
            ui.checkbox(&mut self.st.auto_scale, "Auto");
        });
        ui.add_enabled_ui(!self.st.auto_scale, |ui| {
            row(ui, "Floor", |ui| {
                ui.add(egui::Slider::new(&mut self.st.floor, -140.0..=0.0).suffix(" dB"));
            });
            row(ui, "Ceiling", |ui| {
                ui.add(egui::Slider::new(&mut self.st.ceil, -140.0..=20.0).suffix(" dB"));
            });
        });
    }
}
