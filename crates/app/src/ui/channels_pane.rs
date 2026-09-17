//! What is on each channel: the view a phone Wi-Fi analyser gives, over
//! whatever the decoders say they were on.
//!
//! The rows come from `nodes::ChannelMapNode`, which reads the channel every
//! decode carries. Nothing here knows what 802.11 is: a protocol appears in
//! this pane by saying which channel of which plan it was working, so
//! Bluetooth advertising is listed beside Wi-Fi without a line of pane code.
//!
//! The crowding bar is drawn from the channel a transmitter claims, not the
//! one it was heard on, and counts the neighbours whose width reaches it.
//! That is why channel 3 makes 1 and 6 worse and why a channel with nothing
//! of its own can still be the busiest thing on the band.

use super::*;
use egui::{Color32, Rect, Vec2};

pub(super) struct Channels<'a> {
    pub status: Option<nodes::ChannelStatus>,
    pub filter: &'a mut String,
}

pub(super) enum Action {
    /// Forget every transmitter, for a fresh look at a band.
    Clear,
    /// Tune to this channel's centre.
    Tune(f64),
}

/// The centre of a channel, for the TUNE button.
fn center_hz(plan: common::ChannelPlan, number: u16) -> Option<f64> {
    match plan {
        common::ChannelPlan::Wifi => match number {
            1..=14 => dsp::wifi::channel_2ghz(number as u8),
            32..=196 => Some(dsp::wifi::channel_5ghz(number)),
            _ => None,
        },
        common::ChannelPlan::Ble => {
            dsp::ble::ADV_CHANNELS.iter().find(|(c, _)| u16::from(*c) == number).map(|&(_, hz)| hz)
        }
    }
}

fn cell(ui: &mut egui::Ui, text: &str) {
    theme::Line::new().set(text).size(11.0).show(ui);
}

/// What a row says about the security of a network.
fn protection(s: &common::Secrecy) -> &str {
    match s {
        common::Secrecy::Unsaid => "",
        common::Secrecy::Clear => "open",
        common::Secrecy::Encrypted(Some(name)) => name,
        common::Secrecy::Encrypted(None) => "protected",
    }
}

impl Channels<'_> {
    pub(super) fn show(self, ui: &mut egui::Ui) -> Option<Action> {
        let mut act = None;
        let st = self.status.clone().unwrap_or_default();

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            theme::Line::new()
                .legend("transmitters")
                .value(st.stations.len().to_string())
                .legend("channels")
                .value(st.loads.len().to_string())
                .size(11.0)
                .show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(12.0);
                if !st.stations.is_empty() && ui.button("FORGET").clicked() {
                    act = Some(Action::Clear);
                }
                ui.add(
                    egui::TextEdit::singleline(self.filter)
                        .hint_text("filter")
                        .desired_width(160.0),
                );
            });
        });
        ui.add_space(6.0);

        if st.stations.is_empty() {
            ui.add_space(24.0);
            ui.vertical_centered(|ui| {
                hint(
                    ui,
                    "Nothing has named a channel yet. A transmitter appears here when a decode \
                     says which channel of a plan it was working: 802.11 on 2.4 and 5 GHz, and \
                     Bluetooth LE advertising. The receiver has to be on the band to hear one.",
                );
            });
            return act;
        }

        let now = super::devices_pane::now_us();
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            egui::Frame::NONE.inner_margin(egui::Margin::symmetric(12, 0)).show(ui, |ui| {
                for plan in [common::ChannelPlan::Wifi, common::ChannelPlan::Ble] {
                    let loads: Vec<&nodes::ChannelLoad> =
                        st.loads.iter().filter(|l| l.plan == plan).collect();
                    if loads.is_empty() {
                        continue;
                    }
                    theme::Line::new().legend(plan.label()).size(11.0).show(ui);
                    ui.add_space(2.0);
                    let busiest =
                        loads.iter().map(|l| l.stations + l.overlapping).max().unwrap_or(1).max(1);
                    for l in &loads {
                        if let Some(a) = channel_row(ui, l, busiest, now, &st) {
                            act = Some(a);
                        }
                    }
                    ui.add_space(10.0);
                }
                ui.add_space(4.0);
                self.stations(ui, &st, now);
            });
            ui.add_space(8.0);
        });
        act
    }

    /// One row per transmitter, newest heard first.
    fn stations(&self, ui: &mut egui::Ui, st: &nodes::ChannelStatus, now: u64) {
        let needle = self.filter.to_lowercase();
        let rows: Vec<&nodes::Station> = st
            .stations
            .iter()
            .filter(|s| {
                needle.is_empty()
                    || s.id.to_lowercase().contains(&needle)
                    || s.name.as_deref().unwrap_or("").to_lowercase().contains(&needle)
                    || s.vendor.as_deref().unwrap_or("").to_lowercase().contains(&needle)
            })
            .collect();
        egui::Grid::new("channel_stations").num_columns(7).spacing([14.0, 4.0]).striped(true).show(
            ui,
            |ui| {
                for h in ["address", "name", "channel", "heard on", "security", "level", "seen"] {
                    theme::Line::new().legend(h).size(10.0).show(ui);
                }
                ui.end_row();
                for s in rows {
                    theme::Line::new().heard(s.id.clone()).size(11.0).show(ui);
                    cell(ui, s.name.as_deref().unwrap_or(""));
                    theme::Line::new().value(s.channel.to_string()).size(11.0).show(ui);
                    // Said only when it differs: an access point read off a
                    // filter parked two channels away is the case this pane
                    // exists to show, and repeating the same number on every
                    // other row would bury it.
                    cell(
                        ui,
                        &match s.heard_on == s.channel {
                            true => String::new(),
                            false => s.heard_on.to_string(),
                        },
                    );
                    cell(ui, protection(&s.secrecy));
                    theme::Line::new()
                        .value(format!("{:.0} dBFS", s.rssi_dbfs))
                        .size(11.0)
                        .gap(8.0)
                        .legend(&format!("peak {:.0}", s.best_rssi_dbfs))
                        .size(10.0)
                        .show(ui);
                    theme::Line::new()
                        .value(format!("{} pkt", s.packets))
                        .size(11.0)
                        .gap(8.0)
                        .legend(&super::devices_pane::ago(now.saturating_sub(s.last_us)))
                        .size(10.0)
                        .show(ui);
                    ui.end_row();
                }
            },
        );
    }
}

/// One channel: its number, a bar as long as what is working it, and what
/// reaches it from either side.
fn channel_row(
    ui: &mut egui::Ui,
    l: &nodes::ChannelLoad,
    busiest: u32,
    now: u64,
    st: &nodes::ChannelStatus,
) -> Option<Action> {
    let mut act = None;
    ui.horizontal(|ui| {
        theme::Line::new().legend("ch").value(l.number.to_string()).size(11.0).show(ui);
        let (rect, _) = ui.allocate_exact_size(Vec2::new(180.0, 12.0), egui::Sense::hover());
        bar(ui.painter(), rect, l, busiest);
        theme::Line::new()
            .value(format!("{}", l.stations))
            .size(11.0)
            .legend("on it")
            .size(10.0)
            .gap(10.0)
            .value(format!("{}", l.overlapping))
            .size(11.0)
            .legend("across it")
            .size(10.0)
            .gap(10.0)
            .value(format!("{:.0} dBFS", l.best_rssi_dbfs))
            .size(11.0)
            .show(ui);
        // How recently anything on it was heard, which is what says whether
        // a crowded channel is crowded now.
        if let Some(last) = st
            .stations
            .iter()
            .filter(|s| s.plan == l.plan && s.channel == l.number)
            .map(|s| s.last_us)
            .max()
        {
            theme::Line::new()
                .legend("seen")
                .value(super::devices_pane::ago(now.saturating_sub(last)))
                .size(10.0)
                .show(ui);
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if let Some(hz) = center_hz(l.plan, l.number)
                && ui.button("TUNE").clicked()
            {
                act = Some(Action::Tune(hz));
            }
        });
    });
    act
}

/// The crowding bar: what is working the channel in cyan, what reaches it
/// from a neighbouring channel behind it in the etched colour, both against
/// the busiest channel on the band.
fn bar(p: &egui::Painter, r: Rect, l: &nodes::ChannelLoad, busiest: u32) {
    p.rect_filled(r, 2.0, theme::WELL);
    let share = |n: u32| (n as f32 / busiest as f32).clamp(0.0, 1.0) * r.width();
    let total = share(l.stations + l.overlapping);
    if total > 0.0 {
        let mut back = r;
        back.max.x = r.min.x + total;
        p.rect_filled(back, 2.0, Color32::from_rgb(60, 66, 74));
    }
    let own = share(l.stations);
    if own > 0.0 {
        let mut front = r;
        front.max.x = r.min.x + own;
        p.rect_filled(front, 2.0, theme::TRACE);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_channel_number_resolves_to_the_frequency_a_radio_tunes() {
        assert_eq!(center_hz(common::ChannelPlan::Wifi, 1), Some(2_412e6));
        assert_eq!(center_hz(common::ChannelPlan::Wifi, 11), Some(2_462e6));
        assert_eq!(center_hz(common::ChannelPlan::Wifi, 14), Some(2_484e6));
        assert_eq!(center_hz(common::ChannelPlan::Wifi, 36), Some(5_180e6));
        // The three advertising channels, and nothing for a data channel a
        // receiver was never parked on.
        assert_eq!(center_hz(common::ChannelPlan::Ble, 38), Some(2_426e6));
        assert_eq!(center_hz(common::ChannelPlan::Ble, 12), None);
        assert_eq!(center_hz(common::ChannelPlan::Wifi, 200), None);
    }

    #[test]
    fn a_network_says_what_protects_it_and_an_ordinary_frame_says_nothing() {
        assert_eq!(protection(&common::Secrecy::Unsaid), "");
        assert_eq!(protection(&common::Secrecy::Clear), "open");
        assert_eq!(protection(&common::Secrecy::Encrypted(Some("wpa2".into()))), "wpa2");
        assert_eq!(protection(&common::Secrecy::Encrypted(None)), "protected");
    }
}
