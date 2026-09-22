//! BaseStation lines, the comma separated format dump1090 serves on 30003.
//!
//! A consumer of this port reads decoded fields rather than frames, so the
//! position has to be resolved before a line is written: that is
//! [`crate::track`], and this renders what it holds.

use crate::track::Tracker;
use decode::adsb::{Frame, Message};

/// The lines a frame produces, which is usually one and sometimes none.
pub fn lines(track: &Tracker, f: &Frame, now: chrono::DateTime<chrono::Utc>) -> Vec<String> {
    let Some(icao) = f.icao else { return Vec::new() };
    let hex = format!("{icao:06X}");
    let stamp = now.format("%Y/%m/%d,%H:%M:%S%.3f").to_string();
    // Both halves of every line are the same timestamp: this receiver
    // decodes as it demodulates, so a message is generated and logged at
    // the same moment.
    let head = |kind: u8| format!("MSG,{kind},1,1,{hex},1,{stamp},{stamp}");
    let plane = track.get(icao);
    let at = plane.and_then(|p| p.at);
    let (lat, lon) = match at {
        Some((lat, lon)) => (format!("{lat:.5}"), format!("{lon:.5}")),
        None => (String::new(), String::new()),
    };

    match &f.kind {
        Message::Identification { callsign, .. } => {
            vec![format!("{},{callsign},,,,,,,,,,,0", head(1))]
        }
        Message::AirbornePosition { altitude_ft, .. } => {
            let alt = altitude_ft.map(|a| a.to_string()).unwrap_or_default();
            let call = plane.and_then(|p| p.callsign.clone()).unwrap_or_default();
            vec![format!("{},{call},{alt},,,{lat},{lon},,,,,,0", head(3))]
        }
        Message::SurfacePosition { .. } => {
            vec![format!("{},,,,,{lat},{lon},,,,,,-1", head(3))]
        }
        Message::Velocity { ground_speed_kt, track_deg, vertical_rate_fpm } => {
            vec![format!(
                "{},,,{ground_speed_kt:.0},{track_deg:.0},,,{vertical_rate_fpm},,,,,0",
                head(4)
            )]
        }
        // A reply to an interrogation: an altitude from DF20 and a squawk
        // from DF21, which are transmission types 5 and 6.
        Message::CommB { altitude_ft, squawk, .. } => {
            let mut out = Vec::new();
            if let Some(alt) = altitude_ft {
                out.push(format!("{},,{alt},,,,,,,,,,0", head(5)));
            }
            if let Some(sq) = squawk {
                out.push(format!("{},,,,,,,,{sq:04},,,,0", head(6)));
            }
            out
        }
        // A short reply says only that the aircraft is there, which is
        // transmission type 8 and is what keeps it on a list.
        Message::ShortReply => vec![format!("{},,,,,,,,,,,,0", head(8))],
        Message::Unsupported { .. } => Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::track::Tracker;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }

    /// The fields a BaseStation consumer reads, in the columns it reads them
    /// in.
    ///
    /// A client parses by position, so a line one field short puts
    /// IsOnGround in the SPI column and loses it, which is what these lines
    /// did until the count was pinned: dump1090's net_io.c writes ten fields
    /// of header and twelve of message, twenty-two in all.
    #[test]
    fn every_line_carries_the_twenty_two_fields_dump1090_writes() {
        let mut track = Tracker::default();
        let at = chrono::DateTime::from_timestamp(1_700_000_000, 0).unwrap();
        let mut seen: Vec<(u8, usize)> = Vec::new();
        let mut all = Vec::new();
        for raw in [
            "8D4840D6202CC371C32CE0576098",
            "8D40621D58C386435CC412692AD6",
            "8D40621D58C382D690C8AC2863A7",
            "8D485020994409940838175B284F",
            "A0001838CA380031440000F24184",
        ] {
            let f = decode::adsb::parse(&hex(raw)).expect("a frame");
            track.accept(&f, -12.0);
            for line in lines(&track, &f, at) {
                let fields: Vec<&str> = line.split(',').collect();
                assert_eq!(fields.len(), 22, "{line}");
                assert_eq!(fields[0], "MSG");
                seen.push((fields[1].parse().unwrap(), fields.len()));
                all.push(fields.iter().map(|s| s.to_string()).collect::<Vec<_>>());
            }
        }
        assert_eq!(
            seen.iter().map(|(k, _)| *k).collect::<Vec<u8>>(),
            [1, 3, 3, 4, 5],
            "transmission types off five frames"
        );

        // The identification line, whole.
        assert_eq!(
            all[0].join(","),
            "MSG,1,1,1,4840D6,1,2023/11/14,22:13:20.000,2023/11/14,22:13:20.000,KLM1023,,,,,,,,,,,0"
        );
        // A position needs both halves of its pair, so the first of the two
        // carries none and the second is the worked example's place.
        assert_eq!((all[1][14].as_str(), all[1][15].as_str()), ("", ""));
        assert_eq!((all[2][14].as_str(), all[2][15].as_str()), ("52.25720", "3.91937"));
        assert_eq!(all[2][11], "38000", "feet");
        assert_eq!((all[2][21].as_str(), all[2][20].as_str()), ("0", ""), "on the ground, and SPI");
        // Velocity: ground speed, track and the rate of climb, in theirs.
        assert_eq!((all[3][12].as_str(), all[3][13].as_str()), ("159", "183"));
        assert_eq!(all[3][16], "-832", "feet a minute");
        // And a Comm-B altitude reply, which is transmission type 5.
        assert_eq!(all[4][11], "38000", "feet, off a Comm-B reply");
    }
}
