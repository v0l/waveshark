#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mode {
    pub width: usize,
    pub height: usize,
    pub total_width: usize,
    pub total_height: usize,
    pub refresh_centihz: u32,
    pub pixel_clock_hz: u64,
}

impl Mode {
    pub fn refresh_hz(&self) -> f64 {
        self.refresh_centihz as f64 / 100.0
    }

    pub fn line_hz(&self) -> f64 {
        self.pixel_clock_hz as f64 / self.total_width as f64
    }

    pub fn label(&self) -> String {
        format!("{}x{} {:.0} Hz", self.width, self.height, self.refresh_hz())
    }

    pub fn aspect(&self) -> f32 {
        self.width as f32 / self.height as f32
    }
}

pub fn modes() -> &'static [Mode] {
    &[
        Mode {
            width: 640,
            height: 480,
            total_width: 800,
            total_height: 525,
            refresh_centihz: 5994,
            pixel_clock_hz: 25_175_000,
        },
        Mode {
            width: 640,
            height: 480,
            total_width: 840,
            total_height: 500,
            refresh_centihz: 7500,
            pixel_clock_hz: 31_500_000,
        },
        Mode {
            width: 800,
            height: 600,
            total_width: 1056,
            total_height: 628,
            refresh_centihz: 6032,
            pixel_clock_hz: 40_000_000,
        },
        Mode {
            width: 800,
            height: 600,
            total_width: 1056,
            total_height: 625,
            refresh_centihz: 7500,
            pixel_clock_hz: 49_500_000,
        },
        Mode {
            width: 1024,
            height: 768,
            total_width: 1344,
            total_height: 806,
            refresh_centihz: 6000,
            pixel_clock_hz: 65_000_000,
        },
        Mode {
            width: 1024,
            height: 768,
            total_width: 1312,
            total_height: 800,
            refresh_centihz: 7503,
            pixel_clock_hz: 78_750_000,
        },
        Mode {
            width: 1280,
            height: 720,
            total_width: 1650,
            total_height: 750,
            refresh_centihz: 6000,
            pixel_clock_hz: 74_250_000,
        },
        Mode {
            width: 1280,
            height: 1024,
            total_width: 1688,
            total_height: 1066,
            refresh_centihz: 6002,
            pixel_clock_hz: 108_000_000,
        },
        Mode {
            width: 1280,
            height: 1024,
            total_width: 1688,
            total_height: 1066,
            refresh_centihz: 7502,
            pixel_clock_hz: 135_000_000,
        },
        Mode {
            width: 1366,
            height: 768,
            total_width: 1792,
            total_height: 798,
            refresh_centihz: 5979,
            pixel_clock_hz: 85_500_000,
        },
        Mode {
            width: 1440,
            height: 900,
            total_width: 1904,
            total_height: 934,
            refresh_centihz: 5989,
            pixel_clock_hz: 106_500_000,
        },
        Mode {
            width: 1600,
            height: 900,
            total_width: 1800,
            total_height: 1000,
            refresh_centihz: 6000,
            pixel_clock_hz: 108_000_000,
        },
        Mode {
            width: 1600,
            height: 1200,
            total_width: 2160,
            total_height: 1250,
            refresh_centihz: 6000,
            pixel_clock_hz: 162_000_000,
        },
        Mode {
            width: 1680,
            height: 1050,
            total_width: 2240,
            total_height: 1089,
            refresh_centihz: 5995,
            pixel_clock_hz: 146_250_000,
        },
        Mode {
            width: 1920,
            height: 1080,
            total_width: 2640,
            total_height: 1125,
            refresh_centihz: 5000,
            pixel_clock_hz: 148_500_000,
        },
        Mode {
            width: 1920,
            height: 1080,
            total_width: 2200,
            total_height: 1125,
            refresh_centihz: 6000,
            pixel_clock_hz: 148_500_000,
        },
        Mode {
            width: 1920,
            height: 1200,
            total_width: 2080,
            total_height: 1235,
            refresh_centihz: 5995,
            pixel_clock_hz: 154_000_000,
        },
        Mode {
            width: 2560,
            height: 1440,
            total_width: 2720,
            total_height: 1481,
            refresh_centihz: 5995,
            pixel_clock_hz: 241_500_000,
        },
        Mode {
            width: 3840,
            height: 2160,
            total_width: 4400,
            total_height: 2250,
            refresh_centihz: 6000,
            pixel_clock_hz: 594_000_000,
        },
    ]
}

pub const REFRESH_TOLERANCE_HZ: f64 = 0.1;

pub fn match_mode(refresh_hz: f64, lines: usize) -> Option<&'static Mode> {
    modes()
        .iter()
        .filter(|m| m.total_height == lines)
        .filter(|m| (m.refresh_hz() - refresh_hz).abs() <= REFRESH_TOLERANCE_HZ)
        .min_by(|a, b| {
            let d = |m: &Mode| (m.refresh_hz() - refresh_hz).abs();
            d(a).partial_cmp(&d(b)).unwrap_or(std::cmp::Ordering::Equal)
        })
}

pub fn by_label(label: &str) -> Option<&'static Mode> {
    modes().iter().find(|m| m.label() == label)
}

pub fn labels() -> Vec<String> {
    modes().iter().map(Mode::label).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_mode_agrees_with_its_own_clock() {
        for m in modes() {
            let refresh = m.pixel_clock_hz as f64 / (m.total_width * m.total_height) as f64;
            assert!(
                (refresh - m.refresh_hz()).abs() < 0.02,
                "{} runs at {refresh:.2} Hz off its own clock, not {:.2}",
                m.label(),
                m.refresh_hz()
            );
            assert!(m.total_width > m.width, "{} has no horizontal blanking", m.label());
            assert!(m.total_height > m.height, "{} has no vertical blanking", m.label());
        }
        assert_eq!(modes().len(), 19, "modes in the table");
    }

    #[test]
    fn a_measured_raster_names_the_screen_it_came_off() {
        let named = |hz, lines| match_mode(hz, lines).map(|m| m.label());
        assert_eq!(named(59.94, 525).as_deref(), Some("640x480 60 Hz"));
        assert_eq!(named(60.004, 806).as_deref(), Some("1024x768 60 Hz"));
        assert_eq!(named(60.02, 1066).as_deref(), Some("1280x1024 60 Hz"));
        assert_eq!(named(60.0, 1125).as_deref(), Some("1920x1080 60 Hz"));
        assert_eq!(named(60.0, 2250).as_deref(), Some("3840x2160 60 Hz"));
        assert_eq!(named(50.0, 1125).as_deref(), Some("1920x1080 50 Hz"));
        assert_eq!(named(60.0, 999), None);
        assert_eq!(named(72.0, 1125), None);
    }

    #[test]
    fn a_matched_mode_says_what_clock_to_look_for() {
        let m = match_mode(60.0, 1125).expect("1080p60");
        assert_eq!(m.pixel_clock_hz, 148_500_000);
        assert_eq!(m.width, 1920);
        assert_eq!(m.height, 1080);
        assert!((m.line_hz() - 67_500.0).abs() < 1.0, "{} lines a second", m.line_hz());
        assert!((m.aspect() - 16.0 / 9.0).abs() < 0.001);
    }

    #[test]
    fn a_mode_answers_to_the_label_it_is_offered_under() {
        assert_eq!(by_label("1280x1024 60 Hz").map(|m| m.total_height), Some(1066));
        assert_eq!(by_label("nothing anybody runs"), None);
        assert_eq!(labels().len(), modes().len());
        assert_eq!(labels().iter().filter(|l| l.starts_with("1280x1024")).count(), 2);
    }
}
