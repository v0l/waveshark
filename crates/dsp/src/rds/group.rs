//! Turning RDS groups into station information.
//!
//! Only the groups worth having on a spectrum display are decoded: 0A and 0B
//! carry the eight-character programme service name, 2A and 2B carry
//! radiotext, and every group carries the programme identification code and
//! programme type.

use super::block::Group;

/// Programme type names, index 0 to 31 (European table).
pub const PTY: [&str; 32] = [
    "None",
    "News",
    "Current affairs",
    "Information",
    "Sport",
    "Education",
    "Drama",
    "Culture",
    "Science",
    "Varied",
    "Pop music",
    "Rock music",
    "Easy listening",
    "Light classical",
    "Serious classical",
    "Other music",
    "Weather",
    "Finance",
    "Children",
    "Social affairs",
    "Religion",
    "Phone in",
    "Travel",
    "Leisure",
    "Jazz music",
    "Country music",
    "National music",
    "Oldies music",
    "Folk music",
    "Documentary",
    "Alarm test",
    "Alarm",
];

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Station {
    /// Programme identification, unique per station per country.
    pub pi: Option<u16>,
    pub pty: Option<u8>,
    /// True for a music programme, false for speech.
    pub music: Option<bool>,
    pub traffic_programme: bool,
    /// Eight-character station name, once every segment has arrived.
    pub name: Option<String>,
    pub radiotext: Option<String>,
}

impl Station {
    /// Programme type, or `None` when the station declares none.
    ///
    /// Code zero means the station is not saying, which is not the same as the
    /// string "None" and should not be shown as though it were an answer.
    pub fn pty_name(&self) -> Option<&'static str> {
        match self.pty {
            Some(0) | None => None,
            Some(p) => Some(PTY[(p & 31) as usize]),
        }
    }
}

/// Accumulates groups into a [`Station`].
///
/// The name and radiotext arrive a couple of characters at a time and are only
/// published once every segment has been seen, so a partly filled buffer is
/// never shown as if it were the real name.
pub struct GroupDecoder {
    station: Station,
    name_buf: [u8; 8],
    name_seen: u8,
    rt_buf: [u8; 64],
    rt_seen: u64,
    /// Radiotext A/B flag; a change means the message was replaced.
    rt_ab: Option<bool>,
    rt_len: usize,
}

impl Default for GroupDecoder {
    fn default() -> Self {
        Self {
            station: Station::default(),
            name_buf: [b' '; 8],
            name_seen: 0,
            // Spaces rather than zeros: an unfilled slot should read as blank.
            rt_buf: [b' '; 64],
            rt_seen: 0,
            rt_ab: None,
            rt_len: 0,
        }
    }
}

impl GroupDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn station(&self) -> &Station {
        &self.station
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn push(&mut self, g: &Group) {
        // Blocks A and B are guaranteed by the synchroniser; C and D are not,
        // so anything reading them checks first.
        let pi = g.words[0];
        self.station.pi = Some(pi);

        let b = g.words[1];
        let group_type = (b >> 12) & 0xF;
        let version_b = (b >> 11) & 1 == 1;
        self.station.traffic_programme = (b >> 10) & 1 == 1;
        self.station.pty = Some(((b >> 5) & 0x1F) as u8);

        match (group_type, version_b) {
            (0, _) => self.program_service(g, b, version_b),
            (2, _) => self.radiotext(g, b, version_b),
            _ => {}
        }
    }

    fn program_service(&mut self, g: &Group, b: u16, version_b: bool) {
        self.station.music = Some((b >> 3) & 1 == 1);
        let seg = (b & 0x3) as usize;
        // Version B repeats the PI in block C, so the characters are always in
        // block D regardless of version.
        let _ = version_b;
        if !g.valid[3] {
            return;
        }
        let chars = g.words[3];
        self.name_buf[seg * 2] = (chars >> 8) as u8;
        self.name_buf[seg * 2 + 1] = (chars & 0xFF) as u8;
        self.name_seen |= 3 << (seg * 2);
        if self.name_seen == 0xFF {
            self.station.name = Some(decode_text(&self.name_buf).trim().to_string());
        }
    }

    fn radiotext(&mut self, g: &Group, b: u16, version_b: bool) {
        let ab = (b >> 4) & 1 == 1;
        if self.rt_ab != Some(ab) {
            // A flipped flag means the message is being replaced. Clear the
            // published text as well as the buffer: continuing to show the old
            // one presents it as the current message, which it no longer is.
            // The cost is a blank display while the replacement arrives.
            self.rt_buf = [b' '; 64];
            self.rt_seen = 0;
            self.rt_len = 0;
            self.rt_ab = Some(ab);
            if self.station.radiotext.is_some() {
                self.station.radiotext = None;
            }
        }
        let seg = (b & 0xF) as usize;
        // Version A carries four characters across blocks C and D, version B
        // carries two in block D and reuses block C for the PI.
        let (chars, count, base) = if version_b {
            if !g.valid[3] {
                return;
            }
            ([g.words[3], 0], 2, seg * 2)
        } else {
            if !(g.valid[2] && g.valid[3]) {
                return;
            }
            ([g.words[2], g.words[3]], 4, seg * 4)
        };

        for i in 0..count {
            let w = chars[i / 2];
            let c = if i % 2 == 0 { (w >> 8) as u8 } else { (w & 0xFF) as u8 };
            let idx = base + i;
            if idx >= 64 {
                break;
            }
            if c == 0x0D {
                // Carriage return marks the end of a shorter message.
                self.rt_len = idx;
                self.rt_seen |= (1u64 << idx).wrapping_sub(1);
                continue;
            }
            self.rt_buf[idx] = c;
            self.rt_seen |= 1u64 << idx;
        }

        let want = if self.rt_len > 0 { self.rt_len } else { 64 };
        let mask = if want >= 64 { u64::MAX } else { (1u64 << want) - 1 };
        if self.rt_seen & mask == mask {
            self.station.radiotext = Some(decode_text(&self.rt_buf[..want]).trim_end().to_string());
        }
    }
}

/// RDS uses its own character set; the printable ASCII range matches, and
/// anything else is shown as a space rather than mangling the line.
fn decode_text(bytes: &[u8]) -> String {
    bytes.iter().map(|b| if (0x20..0x7F).contains(b) { *b as char } else { ' ' }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn group(a: u16, b: u16, c: u16, d: u16) -> Group {
        Group { words: [a, b, c, d], valid: [true; 4], c_prime: false }
    }

    /// Build the 0A groups that carry a station name.
    fn name_groups(pi: u16, name: &[u8; 8]) -> Vec<Group> {
        (0..4)
            .map(|seg| {
                // Block B: group 0, version A, TP clear, PTY 9, segment.
                let b = (9 << 5) | seg as u16;
                let d = ((name[seg * 2] as u16) << 8) | name[seg * 2 + 1] as u16;
                group(pi, b, 0, d)
            })
            .collect()
    }

    #[test]
    fn a_station_name_appears_once_every_segment_has_arrived() {
        let mut d = GroupDecoder::new();
        let groups = name_groups(0xF212, b"RADIO 1 ");
        for g in &groups[..3] {
            d.push(g);
            assert!(d.station().name.is_none(), "published a partial name");
        }
        d.push(&groups[3]);
        assert_eq!(d.station().name.as_deref(), Some("RADIO 1"));
    }

    #[test]
    fn the_pi_code_and_programme_type_come_from_every_group() {
        let mut d = GroupDecoder::new();
        d.push(&name_groups(0xC479, b"BBC R4  ")[0]);
        assert_eq!(d.station().pi, Some(0xC479));
        assert_eq!(d.station().pty, Some(9));
        assert_eq!(d.station().pty_name(), Some("Varied"));
    }

    #[test]
    fn a_station_declaring_no_programme_type_reports_none() {
        // Code zero means "not stated". Rendering it as the word "None" puts
        // a value on screen where the station gave one.
        let mut d = GroupDecoder::new();
        let g = group(0xF212, 0 << 5, 0, 0);
        d.push(&g);
        assert_eq!(d.station().pty, Some(0));
        assert_eq!(d.station().pty_name(), None);
    }

    #[test]
    fn segments_arriving_out_of_order_still_assemble() {
        let mut d = GroupDecoder::new();
        let g = name_groups(0xF212, b"CLASSIC ");
        for i in [2usize, 0, 3, 1] {
            d.push(&g[i]);
        }
        assert_eq!(d.station().name.as_deref(), Some("CLASSIC"));
    }

    /// Build 2A radiotext groups for a 64-character message.
    fn rt_groups(pi: u16, text: &str, ab: bool) -> Vec<Group> {
        let mut buf = [b' '; 64];
        for (i, c) in text.bytes().take(64).enumerate() {
            buf[i] = c;
        }
        (0..16)
            .map(|seg| {
                // Block B: group 2, version A, PTY 10, A/B flag, segment.
                let b = (2 << 12) | (10 << 5) | ((ab as u16) << 4) | seg as u16;
                let c = ((buf[seg * 4] as u16) << 8) | buf[seg * 4 + 1] as u16;
                let d = ((buf[seg * 4 + 2] as u16) << 8) | buf[seg * 4 + 3] as u16;
                group(pi, b, c, d)
            })
            .collect()
    }

    #[test]
    fn radiotext_assembles_from_its_sixteen_segments() {
        let mut d = GroupDecoder::new();
        let msg = "Now playing: something with a reasonably long title";
        for g in rt_groups(0xF212, msg, false) {
            d.push(&g);
        }
        assert_eq!(d.station().radiotext.as_deref(), Some(msg));
    }

    #[test]
    fn a_flipped_ab_flag_starts_a_new_message() {
        let mut d = GroupDecoder::new();
        for g in rt_groups(0xF212, "First message", false) {
            d.push(&g);
        }
        assert_eq!(d.station().radiotext.as_deref(), Some("First message"));
        // Only part of the replacement arrives; the old text must not be
        // spliced into it.
        for g in rt_groups(0xF212, "Second message entirely", true).into_iter().take(3) {
            d.push(&g);
        }
        let rt = d.station().radiotext.as_deref().unwrap_or("");
        assert!(!rt.contains("First"), "old message survived the flag change: {rt:?}");
    }

    #[test]
    fn unprintable_characters_do_not_mangle_the_output() {
        let mut d = GroupDecoder::new();
        let mut g = name_groups(0xF212, b"OK\x00\x01TEST");
        for x in g.iter_mut() {
            d.push(x);
        }
        let n = d.station().name.as_deref().unwrap();
        assert!(n.starts_with("OK"), "got {n:?}");
        assert!(n.is_ascii());
    }

    #[test]
    fn unknown_group_types_are_ignored_without_losing_the_pi() {
        let mut d = GroupDecoder::new();
        // Group type 6, which this decoder does not interpret.
        d.push(&group(0xABCD, (6 << 12) | (5 << 5), 0x1111, 0x2222));
        assert_eq!(d.station().pi, Some(0xABCD));
        assert!(d.station().name.is_none());
    }
}
