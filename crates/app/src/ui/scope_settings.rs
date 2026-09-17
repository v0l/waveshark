//! The spectrum, waterfall and scale panels, over the scope's own settings.
//!
//! These belong to the scope rather than to the application: they set what it
//! draws and how, and the panel is the same state the pane reads.

use super::state::ScopeState;
use super::*;
use crate::ui::widgets::{choice, lamp, section, switch};
use dsp::spectrum::Detector;

/// What the panels want done that they cannot do themselves.
pub(super) enum Action {
    /// The span or the bin count changed, so old rows no longer line up.
    ResetWaterfall,
    /// Write what the heatmap holds, in the colours and against the scale
    /// the waterfall is drawn with.
    ExportHeatmap,
}

/// The scope's settings, over the state they change.
pub(super) struct ScopeSettings<'a> {
    pub st: &'a mut ScopeState,
    /// The record, for the centre spur: removing it is a property of the
    /// receiver rather than of the drawing, but it is set beside it.
    pub settings: crate::session::Settings,
    pub rate: f64,
    /// What the heatmap recorder is holding, for the card that writes it out.
    pub heat: Option<crate::heatmap::HeatmapStatus>,
    pub acts: Vec<Action>,
}

impl ScopeSettings<'_> {
    /// The spectrum panel: what the transform is doing, and the scale it is
    /// drawn against.
    pub(super) fn spectrum(&mut self, ui: &mut egui::Ui) {
        section(ui, "transform", "how the span is divided and how often", |ui| {
            let bins = "How finely the span is divided. More bins tell closer signals apart, \
                        and cost time to transform.";
            row_help(ui, "bins", bins, |ui| {
                let mut n = self.st.fft_size;
                let opts = FFTS.iter().map(|v| (*v, v.to_string()));
                if choice(ui, "fft", &mut n, opts) {
                    self.st.fft_size = n;
                    // The same value the record holds and the radio starts
                    // with, so a chosen transform size survives a restart
                    // rather than only living in the running spectrum.
                    self.st.fft = n;
                    self.acts.push(Action::ResetWaterfall);
                }
            });
            reading(ui, "resolution", bin_hint(self.rate, self.st.fft_size));
            row_help(ui, "refresh", "Frames a second the spectrum is worth producing.", |ui| {
                let mut v = self.st.refresh;
                let opts = REFRESH.iter().map(|(n, f)| (*f, format!("{n} fps")));
                if choice(ui, "fps", &mut v, opts) {
                    self.st.refresh = v;
                }
            });
            let what = "What one point of the trace shows out of the transforms behind it. \
                        Sample is the newest of them, average their mean power, peak the \
                        loudest each bin reached. Average is a steady floor; peak finds a \
                        burst shorter than a frame and reads the floor high.";
            row_help(ui, "detector", what, |ui| {
                let mut d = self.st.trace;
                let opts = Detector::ALL.map(|v| (v, v.label().to_string()));
                if choice(ui, "trace", &mut d, opts) {
                    self.st.trace = d;
                }
            });
            let smooth = "How much of the last drawn frame the next one keeps. This acts \
                          on frames after the detector above has already decided what each \
                          one holds.";
            row_help(ui, "smoothing", smooth, |ui| {
                ui.spacing_mut().slider_width = (ui.available_width() - 120.0).max(80.0);
                let slider = egui::Slider::new(&mut self.st.smoothing, 0.02..=1.0);
                ui.add(slider.show_value(false));
                let text = if self.st.smoothing > 0.95 {
                    "off".to_string()
                } else {
                    format!("{:.0}%", (1.0 - self.st.smoothing) * 100.0)
                };
                theme::Line::new().set(text).size(11.0).show(ui);
            });
            let mut dc = self.settings.read(|s| s.dc_block);
            if switch(ui, "centre spur", &mut dc, "remove", "LO leakage at the tuned frequency.") {
                self.settings.edit(|s| s.dc_block = dc);
            }
        });
        ui.add_space(8.0);
        self.scale(ui);
    }

    /// The waterfall panel: how fast it scrolls, and how much it keeps.
    pub(super) fn waterfall(&mut self, ui: &mut egui::Ui) {
        section(ui, "scroll", "how fast it moves and how much it keeps", |ui| {
            row_help(ui, "rate", "Rows a second.", |ui| {
                let mut v = self.st.rows_per_sec;
                let opts = SPEEDS.iter().map(|(n, f)| (*f, format!("{n} rows/s")));
                if choice(ui, "rows", &mut v, opts) {
                    self.st.rows_per_sec = v;
                }
            });
            let what = "What a row shows out of the frames behind it. Peak is what finds a \
                        transmission: a burst of a few milliseconds is in one frame of the \
                        many a row is made of.";
            row_help(ui, "detector", what, |ui| {
                let mut d = self.st.wf_detector;
                let opts = Detector::ALL.map(|v| (v, v.label().to_string()));
                if choice(ui, "wfdet", &mut d, opts) {
                    self.st.wf_detector = d;
                }
            });
            row_help(ui, "history", "Rows kept, scrolled back to with the wheel.", |ui| {
                let mut n = self.st.wf_rows;
                let opts = [256usize, 512, 1024, 2048].map(|v| (v, format!("{v} rows")));
                if choice(ui, "hist", &mut n, opts) {
                    self.st.wf_rows = n;
                    self.st.wf.set_height(n);
                }
            });
            reading(
                ui,
                "holds",
                format!(
                    "{:.0} s at {:.0} rows/s",
                    self.st.wf.height() as f32 / self.st.rows_per_sec,
                    self.st.rows_per_sec
                ),
            );
            let ramp = "The colours a row is drawn in. Every one brightens with the signal, \
                        so a burst is a burst in any of them. A change starts the history \
                        again, because a row is kept as pixels.";
            row_help(ui, "colours", ramp, |ui| {
                let mut r = self.st.ramp;
                let opts = crate::heatmap::Ramp::ALL.map(|v| (v, v.label().to_string()));
                if choice(ui, "ramp", &mut r, opts) {
                    self.st.ramp = r;
                    self.st.wf.set_ramp(r);
                }
            });
            let contrast = "How far below the trace ceiling the hottest colour sits.";
            row_help(ui, "contrast", contrast, |ui| {
                ui.spacing_mut().slider_width = (ui.available_width() - 120.0).max(80.0);
                let slider = egui::Slider::new(&mut self.st.wf_top_offset, 0.0..=20.0);
                ui.add(slider.show_value(false));
                theme::Line::new()
                    .set(format!("{:.0} dB", self.st.wf_top_offset))
                    .size(11.0)
                    .show(ui);
            });
        });
        ui.add_space(8.0);
        self.heatmap(ui);
        ui.add_space(8.0);
        self.scale(ui);
    }

    /// The readings kept behind the waterfall, and the files they make.
    fn heatmap(&mut self, ui: &mut egui::Ui) {
        let (mut on, mut rows, mut cap) =
            self.settings.read(|s| (s.heat_on, s.heat_rows_per_sec, s.heat_cap_mb));
        let was = (on, rows, cap);
        let status = self.heat.clone().unwrap_or_default();
        section(ui, "heatmap", "the span kept as readings, and written out as a file", |ui| {
            let keep = "Decibels per bin, kept apart from the display so an export can be \
                        coloured and scaled afterwards. A retune starts it again: two \
                        tunings are two frequency axes.";
            switch(ui, "record", &mut on, "keep the readings", keep);
            row_help(ui, "rate", "Rows a second kept. Slower holds more of the night.", |ui| {
                let opts = HEAT_ROWS.iter().map(|(n, f)| (*f, (*n).to_string()));
                let mut v = rows;
                if choice(ui, "heat_rows", &mut v, opts) {
                    rows = v;
                }
            });
            row_help(ui, "memory", "What the readings may take before the oldest go.", |ui| {
                let opts = HEAT_CAPS.iter().map(|v| (*v, format!("{v} MB")));
                let mut v = cap;
                if choice(ui, "heat_cap", &mut v, opts) {
                    cap = v;
                }
            });
            reading(
                ui,
                "holding",
                match status.rows {
                    0 => "nothing yet".to_string(),
                    n => format!(
                        "{n} rows, {:.0} s, {:.1} MB",
                        status.seconds,
                        status.bytes as f64 / (1 << 20) as f64
                    ),
                },
            );
            let write = "A page holding the readings themselves: it draws them at the \
                         colours and scale set here, zooms and pans, and gives the time, \
                         the frequency and the decibels under the pointer.";
            row_help(ui, "write", write, |ui| {
                if ui.button("WRITE").clicked() {
                    self.acts.push(Action::ExportHeatmap);
                }
            });
            match (&status.error, &status.saved) {
                (Some(e), _) => lamp(ui, false, e),
                (None, Some(p)) => lamp(ui, true, &p.display().to_string()),
                (None, None) => {}
            }
        });
        if (on, rows, cap) != was {
            self.settings.edit(|s| {
                s.heat_on = on;
                s.heat_rows_per_sec = rows;
                s.heat_cap_mb = cap;
            });
        }
    }

    /// The decibel scale both of them are drawn against.
    fn scale(&mut self, ui: &mut egui::Ui) {
        section(ui, "scale", "the decibels the trace and the colours are drawn against", |ui| {
            switch(
                ui,
                "auto",
                &mut self.st.auto_scale,
                "follow the floor and the peaks",
                "Fitted to what is arriving. Off keeps the floor and ceiling set below.",
            );
            ui.add_enabled_ui(!self.st.auto_scale, |ui| {
                row(ui, "floor", |ui| {
                    ui.spacing_mut().slider_width = (ui.available_width() - 120.0).max(80.0);
                    ui.add(egui::Slider::new(&mut self.st.floor, -140.0..=0.0).suffix(" dB"));
                });
                row(ui, "ceiling", |ui| {
                    ui.spacing_mut().slider_width = (ui.available_width() - 120.0).max(80.0);
                    ui.add(egui::Slider::new(&mut self.st.ceil, -140.0..=20.0).suffix(" dB"));
                });
            });
        });
    }
}
