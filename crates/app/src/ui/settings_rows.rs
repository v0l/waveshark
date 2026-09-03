//! The scanner table as the settings modal edits it.
//!
//! Held as text apart from the live table, so that a half-typed block does
//! not retune the receiver on every keystroke.



/// One scanner as the interface edits it.
///
/// Frequencies are held in the units they are typed in, and the lists stay as
/// text while they are being typed: a half-finished "161.9" must not be
/// parsed into a channel and used to decide what runs. Converting happens at
/// [`ScannerRow::to_scanner`], and a row that does not convert is shown as
/// incomplete rather than silently dropped.
pub struct ScannerRow {
    pub(super) name: String,
    pub(super) lo_mhz: f64,
    pub(super) hi_mhz: f64,
    pub(super) span_khz: f64,
    pub(super) margin_khz: f64,
    pub(super) front: crate::scanners::Front,
    pub(super) channels: String,
    pub(super) widths: String,
    pub(super) enabled: bool,
}

impl ScannerRow {
    pub(super) fn from_scanner(s: &crate::scanners::Scanner) -> Self {
        let widths = match &s.front {
            crate::scanners::Front::Banks(w) => {
                w.iter().map(|x| trim_num(x / 1e3)).collect::<Vec<_>>().join(", ")
            }
            _ => String::new(),
        };
        Self {
            name: s.name.clone(),
            lo_mhz: s.lo / 1e6,
            hi_mhz: s.hi / 1e6,
            span_khz: s.min_rate / 1e3,
            margin_khz: s.margin_hz / 1e3,
            front: s.front.clone(),
            channels: s.channels.iter().map(|c| trim_num(c / 1e6)).collect::<Vec<_>>().join(", "),
            widths,
            enabled: s.enabled,
        }
    }

    /// A new block around the frequency being looked at, which is why
    /// somebody is adding one.
    pub(super) fn new_at(center: f64, rate: f64) -> Self {
        let mhz = center / 1e6;
        let half = (rate / 2e6).max(0.01);
        Self {
            name: "New scanner".into(),
            lo_mhz: (mhz - half).max(0.0),
            hi_mhz: mhz + half,
            span_khz: (rate / 1e3).max(1.0),
            margin_khz: 0.0,
            front: crate::scanners::Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
            channels: String::new(),
            widths: crate::scanners::DEFAULT_WIDTHS
                .iter()
                .map(|x| trim_num(x / 1e3))
                .collect::<Vec<_>>()
                .join(", "),
            enabled: true,
        }
    }

    /// A block prefilled from a packet the log already decoded: the (+) on a
    /// row. The frequency is the row's, and the front end is the one that
    /// reads that protocol, so a page seen once becomes a channel kept. What
    /// the front cannot be inferred from the protocol name falls to `auto`,
    /// which decodes it wherever it lands anyway.
    pub(super) fn for_packet(freq: f64, model: &str, rate: f64) -> Self {
        use crate::scanners::Front;
        let system = model.split('-').next().unwrap_or(model);
        let front = match system {
            "POCSAG" => Front::Pocsag(freq),
            "M17" => Front::M17(freq),
            "APRS" | "AX25" => Front::Aprs(freq),
            "ModeS" => Front::ModeS,
            "AIS" => Front::Ais,
            _ => Front::Auto,
        };
        // A per-channel front is a pin on the frequency; a band front is
        // about the span the packet arrived in, so it keeps a range.
        if front.per_channel() {
            Self::pinned_at(freq, front)
        } else {
            let mut row = Self::new_at(freq, rate);
            row.name = format!("{} {:.4} MHz", front.label(), freq / 1e6);
            row.front = front;
            row
        }
    }

    /// A fixed decode channel on the dial: one per-channel front end (pager,
    /// M17 or APRS) pinned to the frequency being looked at, the counterpart
    /// of the audio channel strip for the things that decode rather than
    /// play. The channel is the dial, the range is a narrow window around it
    /// so the block still reads coherently if its front is later switched to
    /// a band one, and the name carries the frequency so a list of pins is
    /// legible.
    pub(super) fn pinned_at(center: f64, front: crate::scanners::Front) -> Self {
        let mhz = center / 1e6;
        Self {
            name: format!("{} {:.4} MHz", front.label(), mhz),
            lo_mhz: (mhz - 0.05).max(0.0),
            hi_mhz: mhz + 0.05,
            // Low enough that any span the operator runs covers it, since a
            // pin is a channel to keep whatever the receiver is set to.
            span_khz: 25.0,
            margin_khz: 0.0,
            channels: trim_num(mhz),
            widths: crate::scanners::DEFAULT_WIDTHS
                .iter()
                .map(|x| trim_num(x / 1e3))
                .collect::<Vec<_>>()
                .join(", "),
            front,
            enabled: true,
        }
    }

    /// The banks front end carrying whatever widths are currently typed, so
    /// switching away and back does not lose them.
    pub(super) fn banks_with_current_widths(&self) -> crate::scanners::Front {
        let w: Vec<f64> = parse_list(&self.widths, 1e3);
        crate::scanners::Front::Banks(if w.is_empty() {
            crate::scanners::DEFAULT_WIDTHS.to_vec()
        } else {
            w
        })
    }

    pub(super) fn to_scanner(&self) -> Option<crate::scanners::Scanner> {
        let name = self.name.trim();
        if name.is_empty() || self.hi_mhz <= self.lo_mhz {
            return None;
        }
        let front = match self.front {
            crate::scanners::Front::Banks(_) => self.banks_with_current_widths(),
            ref f => f.clone(),
        };
        Some(crate::scanners::Scanner {
            name: name.to_string(),
            lo: self.lo_mhz * 1e6,
            hi: self.hi_mhz * 1e6,
            min_rate: self.span_khz * 1e3,
            channels: parse_list(&self.channels, 1e6),
            margin_hz: self.margin_khz * 1e3,
            front,
            enabled: self.enabled,
        })
    }
}

/// A comma separated list of numbers, scaled to hertz.
pub(super) fn parse_list(text: &str, unit: f64) -> Vec<f64> {
    text.split(',').filter_map(|p| p.trim().parse::<f64>().ok()).map(|v| v * unit).collect()
}

/// A number without trailing zeros, for putting one back in a text field.
pub(super) fn trim_num(v: f64) -> String {
    let s = format!("{v:.4}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// A megahertz field, typed to enough places for a 25 kHz channel raster.
pub(super) fn mhz_field(ui: &mut egui::Ui, v: &mut f64) {
    ui.add(egui::DragValue::new(v).speed(0.01).range(0.0..=6000.0).max_decimals(4));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scanners::Front;

    /// A pinned channel becomes a valid single-channel block on the dial: the
    /// frequency is its one channel, the front end is carried through, and a
    /// span narrow enough that whatever the receiver is set to covers it.
    #[test]
    fn a_pin_is_a_single_channel_block_on_the_dial() {
        let row = ScannerRow::pinned_at(439_987_500.0, Front::Pocsag(439_987_500.0));
        let s = row.to_scanner().expect("a pin converts");
        assert_eq!(s.channels, vec![439_987_500.0]);
        assert!(matches!(s.front, Front::Pocsag(_)));
        assert!(s.min_rate <= 25_000.0, "a pin runs at any span, got {}", s.min_rate);
        assert!(s.applies(439_987_500.0, 2_400_000.0), "a pin runs when tuned to it");
    }

    /// The (+) on a packet row picks the front end from the protocol name and
    /// pins it to the row's frequency.
    #[test]
    fn a_packet_row_prefills_the_right_front() {
        let pager = ScannerRow::for_packet(439_987_500.0, "POCSAG-Alpha", 2_400_000.0)
            .to_scanner()
            .expect("pager converts");
        assert!(matches!(pager.front, Front::Pocsag(_)));
        assert_eq!(pager.channels, vec![439_987_500.0]);

        let m17 = ScannerRow::for_packet(433_475_000.0, "M17-Voice", 2_400_000.0);
        assert!(matches!(m17.front, Front::M17(_)));

        // A band protocol keeps a range rather than becoming a channel pin.
        let modes = ScannerRow::for_packet(1_090_000_000.0, "ModeS-Reply", 2_400_000.0);
        assert!(matches!(modes.front, Front::ModeS));
        assert!(modes.hi_mhz > modes.lo_mhz);

        // An unknown burst has no protocol, so it falls to auto.
        let unknown = ScannerRow::for_packet(433_000_000.0, "unknown", 2_400_000.0);
        assert!(matches!(unknown.front, Front::Auto));
    }
}
