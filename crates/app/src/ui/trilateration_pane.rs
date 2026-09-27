use super::devices_pane::{ago, now_us};
use super::state::{SurveyState, Within};
use super::*;

pub(super) struct Trilateration<'a> {
    pub st: &'a mut SurveyState,
    pub recording: bool,
}

pub(super) enum Action {
    ShowOnMap(i64),
}

impl Trilateration<'_> {
    pub(super) fn show(self, ui: &mut egui::Ui) -> Option<Action> {
        let mut act = None;
        let located = self.st.located();
        let within = self.st.within;
        let rows: Vec<&survey::Located> =
            located.iter().filter(|l| within.holds(l.estimate.radius_m)).collect();

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            Line::new()
                .legend("placed")
                .value(format!("{} of {}", rows.len(), located.len()))
                .size(11.0)
                .show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                ui.allocate_ui(egui::vec2(90.0, 20.0), |ui| {
                    egui_bench::form::choice(
                        ui,
                        "trilateration within",
                        &mut self.st.within,
                        Within::ALL.map(|w| (w, w.label().to_string())),
                    );
                });
                Line::new().legend("within").size(11.0).show(ui);
            });
        });
        ui.add_space(6.0);

        let empty = if !self.recording {
            Some(
                "No survey is being recorded. Switch Record on in Devices, or start one with \
                 --survey, and drive round what you want placed.",
            )
        } else if located.is_empty() {
            Some(
                "Nothing has been heard from enough places yet. A transmitter is placed once \
                 the receiver has heard it with a position fix from spots a few hundred metres \
                 apart, on more than one side of it.",
            )
        } else if rows.is_empty() {
            Some("Nothing is placed that tightly yet. Widen the radius above to see the rest.")
        } else {
            None
        };
        if let Some(text) = empty {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| hint(ui, text));
            return act;
        }

        let now_us = now_us();
        let row_h = egui_bench::table::ROW_H + 4.0;
        let (head, _) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), row_h), egui::Sense::hover());
        let head = head.shrink2(egui::vec2(12.0, 0.0));
        egui_bench::table::header(ui.painter(), head, &COLUMNS);
        egui::ScrollArea::vertical().auto_shrink([false, false]).show_rows(
            ui,
            row_h,
            rows.len(),
            |ui, range| {
                for i in range {
                    let (row, _) = ui.allocate_exact_size(
                        egui::vec2(ui.available_width(), row_h),
                        egui::Sense::hover(),
                    );
                    let row = row.shrink2(egui::vec2(12.0, 0.0));
                    let p = ui.painter();
                    if i % 2 == 1 {
                        p.rect_filled(row, 0.0, theme::WELL.gamma_multiply(0.6));
                    }
                    let (d, e) = (&rows[i].device, &rows[i].estimate);
                    let cells = [
                        (d.ident.clone(), theme::TRACE),
                        (d.protocol.clone(), theme::VALUE),
                        (
                            d.name.as_deref().or(d.vendor.as_deref()).unwrap_or("").to_string(),
                            theme::VALUE,
                        ),
                        (format!("{:.5}, {:.5}", e.lat, e.lon), theme::VALUE),
                        (format!("{:.0} m", e.radius_m), theme::READOUT),
                        (e.sightings.to_string(), theme::VALUE),
                        (format!("{:.1} dB", e.residual_db), theme::VALUE),
                        (ago(now_us.saturating_sub(d.last_us)), theme::LEGEND),
                    ];
                    let mut x = row.left() + 6.0;
                    for ((text, tint), (_, w)) in cells.iter().zip(COLUMNS) {
                        egui_bench::table::cell(p, row, x, w, text, *tint);
                        x += w;
                    }
                    let at = egui::Rect::from_min_size(
                        egui::pos2(x, row.top() + 1.0),
                        egui::vec2(44.0, row_h - 2.0),
                    );
                    if ui.put(at, egui::Button::new("MAP").small()).clicked() {
                        act = Some(Action::ShowOnMap(d.id));
                    }
                }
            },
        );
        act
    }
}

const COLUMNS: [(&str, f32); 9] = [
    ("identity", 150.0),
    ("protocol", 70.0),
    ("name", 170.0),
    ("position", 150.0),
    ("within", 70.0),
    ("sightings", 76.0),
    ("fit", 64.0),
    ("seen", 50.0),
    ("", 50.0),
];
