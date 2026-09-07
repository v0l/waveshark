//! The 5.8 GHz analogue FPV channel plan.
//!
//! There is no standard here, only convention. Five bands accumulated from
//! different manufacturers, they overlap each other, and a transmitter is set
//! by a band letter and a channel number printed on a card. Naming the
//! channel a carrier sits on is most of what a receiver can say about an
//! analogue link before it demodulates it, and it is what a pilot means by
//! "who is on 5800".
//!
//! Frequencies as every transmitter's manual lists them, including the
//! duplicates: R1 and F1 are both 5658 MHz, A5 and R2 are both 5732, and so
//! on. A frequency that names two channels names them both here rather than
//! this file picking a favourite.

/// A band as the manuals letter them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Band {
    /// Boscam A, and the TBS/ImmersionRC sets that follow it.
    A,
    /// Boscam B.
    B,
    /// Boscam E, which reaches below the 5.8 GHz ISM allocation.
    E,
    /// Fatshark, also sold as Airwave and IRC.
    F,
    /// Raceband, the one spaced to interfere least with itself.
    R,
}

impl Band {
    pub fn letter(self) -> char {
        match self {
            Self::A => 'A',
            Self::B => 'B',
            Self::E => 'E',
            Self::F => 'F',
            Self::R => 'R',
        }
    }
}

/// Eight channels per band, in megahertz, in the order the manuals number
/// them.
const PLAN: [(Band, [u16; 8]); 5] = [
    (Band::A, [5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725]),
    (Band::B, [5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866]),
    (Band::E, [5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945]),
    (Band::F, [5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880]),
    (Band::R, [5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917]),
];

/// One channel of the plan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Channel {
    pub band: Band,
    /// 1 to 8, as printed.
    pub number: u8,
    pub hz: u64,
}

impl std::fmt::Display for Channel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}{}", self.band.letter(), self.number)
    }
}

/// Every channel in the plan.
pub fn channels() -> Vec<Channel> {
    PLAN.iter()
        .flat_map(|(band, freqs)| {
            freqs.iter().enumerate().map(move |(i, &mhz)| Channel {
                band: *band,
                number: i as u8 + 1,
                hz: u64::from(mhz) * 1_000_000,
            })
        })
        .collect()
}

/// The channels a frequency names, nearest first.
///
/// Plural because the bands overlap: eleven frequencies in the plan carry two
/// names and a receiver cannot tell which the transmitter was set to, since
/// nothing in an analogue signal says. Reporting one of them would be
/// inventing a fact.
pub fn channels_at(hz: u64, tolerance_hz: u64) -> Vec<Channel> {
    let mut near: Vec<Channel> = channels()
        .into_iter()
        .filter(|c| c.hz.abs_diff(hz) <= tolerance_hz)
        .collect();
    near.sort_by_key(|c| c.hz.abs_diff(hz));
    near
}

/// How the channels a frequency names should be written in a row: "R1 or F1"
/// where two share it.
pub fn name_at(hz: u64, tolerance_hz: u64) -> Option<String> {
    let near = channels_at(hz, tolerance_hz);
    (!near.is_empty()).then(|| {
        near.iter()
            .map(|c| c.to_string())
            .collect::<Vec<_>>()
            .join(" or ")
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plan_has_forty_channels_in_five_bands() {
        let all = channels();
        assert_eq!(all.len(), 40);
        assert_eq!(all.iter().filter(|c| c.band == Band::R).count(), 8);
        assert_eq!(
            all.iter().find(|c| c.band == Band::R && c.number == 1).unwrap().hz,
            5_658_000_000
        );
    }

    /// The bands overlap, and a receiver has no way to tell which name the
    /// transmitter was set to. Both are reported.
    #[test]
    fn a_shared_frequency_names_both_channels() {
        let at = name_at(5_732_000_000, 2_000_000).expect("a channel");
        assert!(at.contains("R3"), "{at}");
        assert!(at.contains("B1"), "{at}");

        // 5800 is F4 and nothing else, which is why it is the one everybody
        // says out loud.
        assert_eq!(name_at(5_800_000_000, 1_000_000).as_deref(), Some("F4"));
    }

    #[test]
    fn a_frequency_outside_the_plan_names_nothing() {
        assert_eq!(name_at(5_500_000_000, 2_000_000), None);
        assert_eq!(name_at(2_440_000_000, 2_000_000), None);
    }

    /// A carrier a little off nominal still names its channel: a cheap
    /// transmitter is tens of kilohertz out and the plan is spaced in
    /// megahertz.
    #[test]
    fn a_carrier_slightly_off_still_names_its_channel() {
        assert_eq!(
            name_at(5_800_300_000, 1_000_000).as_deref(),
            Some("F4"),
            "300 kHz of error is a normal transmitter"
        );
    }
}
