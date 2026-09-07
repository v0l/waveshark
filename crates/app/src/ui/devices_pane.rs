//! The device view: what is out there, rather than what was transmitted.
//!
//! The packet list is a stream and reading it while driving is impossible: a
//! busy band puts a hundred rows a second through it and the same beacon is
//! most of them. This is the other half of the same evidence, one row per
//! transmitter, sorted by when it was last heard, with the place it was heard
//! strongest and how many receptions there were.
//!
//! Selecting a row draws that device's sightings on the map, which is the
//! only honest way to show where something is: a line of positions the
//! receiver drove along with the level at each, rather than a pin claiming a
//! coordinate nothing measured.

use super::state::SurveyState;
use super::*;

pub(super) struct Devices<'a> {
    pub st: &'a mut SurveyState,
    /// What the survey holds and what the GPS is doing, from the radio
    /// thread's status.
    pub counts: (u64, u64, u64),
    pub fix: Option<gps::Fix>,
    pub gps_connected: bool,
}

pub(super) enum Action {
    /// Show this device's sightings on the map, or none.
    Select(Option<i64>),
    /// Write the survey out as WiGLE CSV, beside the survey file.
    Export,
}

impl Devices<'_> {
    pub(super) fn show(self, ui: &mut egui::Ui) -> Option<Action> {
        let mut act = None;
        let (devices, sightings, heard) = self.counts;

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            theme::Line::new()
                .legend("devices")
                .value(format!("{devices} heard"))
                .legend("sightings")
                .value(sightings.to_string())
                .legend("receptions")
                .value(heard.to_string())
                .size(11.0)
                .show(ui);
            ui.add_space(12.0);
            // What the position column means, said once at the top rather
            // than implied by empty cells: a survey with no fix is still
            // recording, and an operator should be able to see which it is.
            let (legend, value) = match (self.gps_connected, self.fix) {
                (_, Some(f)) => ("fix", format!("{:.5}, {:.5}", f.lat, f.lon)),
                (true, None) => ("gps", "connected, no fix".into()),
                (false, None) => ("gps", "nothing answering".into()),
            };
            theme::Line::new().legend(legend).value(value).size(11.0).show(ui);
            // Where the selected device's sightings put it. A conclusion
            // drawn from the levels along the drive, with how far it might
            // be out, which is what makes it worth saying at all.
            if self.st.selected.is_some() {
                ui.add_space(12.0);
                let (legend, value) = match &self.st.estimate {
                    Some(e) => (
                        "likely at",
                        format!(
                            "{:.5}, {:.5} within {:.0} m, from {} sightings",
                            e.lat, e.lon, e.radius_m, e.sightings
                        ),
                    ),
                    None => ("likely at", "not enough places heard from yet".into()),
                };
                theme::Line::new().legend(legend).value(value).size(11.0).show(ui);
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                if devices > 0 && ui.button("Export CSV").clicked() {
                    act = Some(Action::Export);
                }
                ui.add(
                    egui::TextEdit::singleline(&mut self.st.filter)
                        .hint_text("filter")
                        .desired_width(160.0),
                );
            });
        });
        ui.add_space(6.0);

        if self.st.path.is_none() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "No survey is being recorded. Start one with --survey, or turn it on in \
                     settings. Every transmitter that identifies itself is recorded once, with \
                     the places it was heard from.",
                );
            });
            return act;
        }
        if self.st.rows.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "Nothing has identified itself yet. A device is recorded when a decode \
                     names its transmitter: a BLE address, an ICAO address, an MMSI, a \
                     callsign, a sensor id.",
                );
            });
            return act;
        }

        let needle = self.st.filter.to_lowercase();
        let rows: Vec<survey::Device> = self
            .st
            .rows
            .iter()
            .filter(|d| {
                needle.is_empty()
                    || d.ident.to_lowercase().contains(&needle)
                    || d.protocol.to_lowercase().contains(&needle)
                    || d.name.as_deref().unwrap_or("").to_lowercase().contains(&needle)
                    || d.vendor.as_deref().unwrap_or("").to_lowercase().contains(&needle)
            })
            .cloned()
            .collect();

        let now_us = now_us();
        let selected = self.st.selected;
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                egui::Grid::new("devices")
                    .num_columns(7)
                    .spacing([14.0, 4.0])
                    .striped(true)
                    .show(ui, |ui| {
                        for h in
                            ["identity", "protocol", "name", "vendor", "level", "seen", "packets"]
                        {
                            ui.label(egui::RichText::new(h).size(10.0).color(theme::LEGEND));
                        }
                        ui.end_row();
                        for d in &rows {
                            let hit = Some(d.id) == selected;
                            let ident = egui::RichText::new(&d.ident)
                                .size(11.0)
                                .color(if hit { theme::READOUT } else { theme::TRACE });
                            if ui.selectable_label(hit, ident).clicked() {
                                act = Some(Action::Select(if hit { None } else { Some(d.id) }));
                            }
                            cell(ui, &d.protocol);
                            cell(ui, d.name.as_deref().unwrap_or(""));
                            cell(ui, d.vendor.as_deref().unwrap_or(""));
                            cell(
                                ui,
                                &d.best_rssi_dbfs
                                    .map(|v| format!("{v:.0} dBFS"))
                                    .unwrap_or_default(),
                            );
                            cell(ui, &ago(now_us.saturating_sub(d.last_us)));
                            let freq = egui::RichText::new(format!(
                                "{} at {:.3} MHz",
                                d.packets,
                                d.center_hz as f64 / 1e6
                            ))
                            .size(11.0)
                            .color(theme::LEGEND);
                            // A reading, not a control: a device list is for
                            // what has been heard, and clicking a row to
                            // retune took the receiver off the band it was
                            // surveying.
                            ui.label(freq);
                            ui.end_row();
                        }
                    });
            });
            ui.add_space(8.0);
        });
        act
    }
}

fn cell(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).size(11.0).color(theme::READOUT));
}

/// Wall clock in microseconds, which is what the survey stamps sightings
/// with: the database outlives the process, so its times cannot be an
/// `Instant`.
pub(super) fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// How long ago, in the space a column has.
fn ago(us: u64) -> String {
    let s = us / 1_000_000;
    match s {
        0..=59 => format!("{s}s"),
        60..=3_599 => format!("{}m", s / 60),
        3_600..=86_399 => format!("{}h", s / 3_600),
        _ => format!("{}d", s / 86_400),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_age_reads_at_the_width_a_column_has() {
        assert_eq!(ago(3_000_000), "3s");
        assert_eq!(ago(300_000_000), "5m");
        assert_eq!(ago(7_200_000_000), "2h");
        assert_eq!(ago(172_800_000_000), "2d");
    }
}
