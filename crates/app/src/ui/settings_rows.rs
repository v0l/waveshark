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
