//! What a burst was keyed on.
//!
//! The classifier decides it, the packet log stores it, the burst pane draws
//! it and the signal identifier looks it up, so it lives here rather than in
//! `dsp`: a verdict passed around as its own label is a set of spellings
//! nothing can check.

/// What the burst was keyed on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Modulation {
    /// Amplitude keyed all the way off. The OOK front end reads these.
    Ook,
    /// Amplitude keyed, but not to zero. Shallow ASK, which needs
    /// the shallow-ASK path rather than the plain envelope path.
    Ask,
    /// Two tones. The FSK front end reads these.
    Fsk2,
    /// Four levels. The four-level front end reads these.
    Fsk4,
    /// Two tones at a modulation index near 0.5, which is MSK and its
    /// filtered relative GMSK. Worth separating from plain FSK because the
    /// tones overlap and a hard threshold on the discriminator loses to a
    /// matched receiver by several dB.
    Msk,
    /// Phase keyed, two states.
    Psk2,
    /// Phase keyed, four states.
    Psk4,
    /// Four phases shifted by an eighth of a turn every symbol, which is
    /// what TETRA and the trunked systems key. The fourth power alternates
    /// sign each symbol, so instead of one line it shows a pair a symbol
    /// rate apart.
    Dqpsk,
    /// Frequency swept linearly, which is chirp spread spectrum and radar.
    Chirp,
    /// Many carriers with a cyclic prefix. Told from the rest of the
    /// noise-like family by the prefix repeating at one lag.
    Ofdm,
    /// A single carrier spread by a chip sequence, which repeats in the
    /// envelope even though the data cancels it in the complex samples.
    Dsss,
    /// Modulated, but with no keying structure to find: flat spectrum,
    /// Gaussian amplitude. OFDM and direct sequence spread spectrum both land
    /// here, and so does interference.
    NoiseLike,
    /// Present and steady. An unmodulated carrier, a leaking oscillator, or
    /// the quiet half of a signal whose data has not started yet.
    Carrier,
    /// Measured, and nothing fit.
    #[default]
    Unknown,
    /// Two tones in an audio channel, which is what a packet radio keys
    /// through an FM receiver.
    ///
    /// This and the four below the classifier never returns: it works on RF
    /// and cannot tell a Gaussian filter from a plain one, let alone that a
    /// signal was keyed at audio. They exist because a demodulator that was
    /// built for one of them knows, and a packet list saying `AFSK` for an
    /// APRS frame is saying something true that `2-FSK` would not.
    Afsk,
    /// Frequency keyed through a Gaussian filter, as Bluetooth LE keys.
    Gfsk,
    /// Minimum shift keying through a Gaussian filter, as GSM and AIS key.
    Gmsk,
    /// Chirp spread spectrum as LoRa and ELRS name it.
    Css,
    /// Pulse position keying, which is how Mode S carries its bits.
    Ppm,
    /// Frequency modulated speech, which is what a demodulated voice call
    /// was carried by.
    Fm,
}

impl Modulation {
    pub fn label(&self) -> &'static str {
        match self {
            Modulation::Ook => "OOK",
            Modulation::Ask => "ASK",
            Modulation::Fsk2 => "2-FSK",
            Modulation::Fsk4 => "4-FSK",
            Modulation::Msk => "MSK",
            Modulation::Psk2 => "BPSK",
            Modulation::Psk4 => "QPSK",
            Modulation::Dqpsk => "pi/4-DQPSK",
            Modulation::Chirp => "chirp",
            Modulation::Ofdm => "OFDM",
            Modulation::Dsss => "DSSS",
            Modulation::NoiseLike => "noise-like",
            Modulation::Carrier => "carrier",
            Modulation::Unknown => "unknown",
            Modulation::Afsk => "AFSK",
            Modulation::Gfsk => "GFSK",
            Modulation::Gmsk => "GMSK",
            Modulation::Css => "CSS",
            Modulation::Ppm => "PPM",
            Modulation::Fm => "FM",
        }
    }

    /// The keying this is a kind of, for a comparison that should not care
    /// about the filter: GFSK is FSK, GMSK is MSK, CSS is a chirp.
    pub fn family(self) -> Self {
        match self {
            Self::Gfsk | Self::Afsk => Self::Fsk2,
            Self::Gmsk => Self::Msk,
            Self::Css => Self::Chirp,
            other => other,
        }
    }

    /// Whether a front end in this crate can read it today.
    pub fn has_front_end(&self) -> bool {
        matches!(self, Modulation::Ook | Modulation::Ask | Modulation::Fsk2 | Modulation::Fsk4)
    }

    /// Whether the verdict says anything about the signal. Noise-like is
    /// the classifier reporting that it found power and no structure in
    /// it, which names nothing and cannot be gone looking for: a list of
    /// those is a list of squelch gates opening, and it buries the bursts
    /// that were worth a row.
    pub fn is_named(&self) -> bool {
        !matches!(self, Modulation::NoiseLike | Modulation::Unknown)
    }

    /// The verdict a label names, for reading a stored one back.
    ///
    /// The labels are what the packet log and the CSV carry, so this has to
    /// stay in step with [`Modulation::label`]; the test beside it walks
    /// every variant.
    pub fn from_label(s: &str) -> Option<Self> {
        Self::ALL.iter().copied().find(|m| m.label() == s)
    }

    /// What a person or a saved patch calls a keying.
    ///
    /// Looser than [`Modulation::from_label`] because the spellings in the
    /// wild are not the classifier's: a decoder says `4FSK` for what the
    /// classifier calls `4-FSK`, and `CSS` and `chirp` are the same sweep
    /// under two names.
    pub fn parse(s: &str) -> Option<Self> {
        let k: String =
            s.chars().filter(char::is_ascii_alphanumeric).collect::<String>().to_ascii_uppercase();
        match k.as_str() {
            "OOK" => Some(Self::Ook),
            "ASK" => Some(Self::Ask),
            "FSK" | "2FSK" | "FSK2" => Some(Self::Fsk2),
            "AFSK" => Some(Self::Afsk),
            "GFSK" => Some(Self::Gfsk),
            "4FSK" | "FSK4" | "C4FM" => Some(Self::Fsk4),
            "MSK" => Some(Self::Msk),
            "GMSK" => Some(Self::Gmsk),
            "BPSK" | "PSK2" => Some(Self::Psk2),
            "QPSK" | "PSK4" => Some(Self::Psk4),
            "PI4DQPSK" | "DQPSK" => Some(Self::Dqpsk),
            "CHIRP" => Some(Self::Chirp),
            "CSS" | "LORA" => Some(Self::Css),
            "PPM" => Some(Self::Ppm),
            "FM" => Some(Self::Fm),
            "OFDM" => Some(Self::Ofdm),
            "DSSS" => Some(Self::Dsss),
            "NOISELIKE" => Some(Self::NoiseLike),
            "CARRIER" => Some(Self::Carrier),
            "UNKNOWN" => Some(Self::Unknown),
            _ => None,
        }
    }

    /// Every verdict, so a caller can round-trip or list them without
    /// keeping its own copy of the set.
    pub const ALL: [Self; 20] = [
        Self::Ook,
        Self::Ask,
        Self::Fsk2,
        Self::Fsk4,
        Self::Msk,
        Self::Psk2,
        Self::Psk4,
        Self::Dqpsk,
        Self::Chirp,
        Self::Ofdm,
        Self::Dsss,
        Self::NoiseLike,
        Self::Carrier,
        Self::Unknown,
        Self::Afsk,
        Self::Gfsk,
        Self::Gmsk,
        Self::Css,
        Self::Ppm,
        Self::Fm,
    ];
}

impl std::fmt::Display for Modulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_verdict_reads_back_from_the_label_it_was_written_as() {
        for m in Modulation::ALL {
            assert_eq!(Modulation::from_label(m.label()), Some(m), "{m}");
        }
        assert_eq!(Modulation::from_label("2FSK"), None);
        assert_eq!(Modulation::from_label(""), None);
        for m in Modulation::ALL {
            assert_eq!(Modulation::parse(m.label()), Some(m), "{m}");
        }
        // The spellings the decoders and the saved patches use.
        assert_eq!(Modulation::parse("FSK"), Some(Modulation::Fsk2));
        assert_eq!(Modulation::parse("4FSK"), Some(Modulation::Fsk4));
        assert_eq!(Modulation::parse("CSS"), Some(Modulation::Css));
        assert_eq!(Modulation::parse("nonsense"), None);
        // The filtered kinds compare as the keying they are.
        assert_eq!(Modulation::Gfsk.family(), Modulation::Fsk2);
        assert_eq!(Modulation::Gmsk.family(), Modulation::Msk);
        assert_eq!(Modulation::Css.family(), Modulation::Chirp);
        assert_eq!(Modulation::Ook.family(), Modulation::Ook);
    }
}
