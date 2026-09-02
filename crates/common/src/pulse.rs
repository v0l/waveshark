//! Pulse-train vocabulary.
//!
//! These types live in `common` rather than `dsp` because they are carried on
//! graph edges, and the graph must not depend on the DSP implementation that
//! happens to produce them. The detector lives in `dsp`; the shape of what it
//! emits belongs to everybody.

use crate::C32;

/// One mark/gap pair, in microseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pulse {
    /// Carrier present, in microseconds.
    pub mark: u32,
    /// Carrier absent, in microseconds. The final gap of a package is the
    /// timeout that ended it and carries no information.
    pub gap: u32,
}

/// A complete burst: the pulses between two long silences.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Package {
    pub pulses: Vec<Pulse>,
    /// Estimated SNR of the burst, in dB.
    pub snr_db: f32,
    /// Received level in dB, referenced to a full scale sample *at the
    /// detector's input*.
    ///
    /// Worth having next to the SNR rather than instead of it: a strong packet
    /// in a noisy channel and a weak one in a quiet channel can share an SNR,
    /// and only the level tells them apart or says the front end is clipping.
    ///
    /// Not calibrated to the antenna, and not quite calibrated to the ADC
    /// either: every filter between the two has gain, so a very strong signal
    /// reads a little above zero rather than pinning at it. Measured on the
    /// Fine Offset capture through a channelizer it comes out at +1 dB. It is
    /// a comparable number between packets on one receiver, which is what it
    /// is for, and not a field strength.
    pub rssi_dbfs: f32,
    /// Sample index where the burst started, for correlating with a waterfall.
    pub start_sample: u64,
    /// Where the burst was received, in Hz.
    ///
    /// Stamped by the detector from the stream it read, which in a channel
    /// bank is the channel's own centre rather than the tuner's. A package
    /// that has been separated from the port it arrived on is otherwise
    /// unplaceable, and a burst without a frequency is not evidence of much.
    pub center_hz: u64,
    /// What the burst was measured to be keyed on, when something measured it.
    ///
    /// `None` where the front end was chosen in advance rather than from the
    /// signal, which is every chain that runs one demodulator by
    /// configuration. A guess from the channel width stands in there, and it
    /// is a guess: a 125 kHz channel holds an OOK sensor as readily as an FSK
    /// one.
    pub modulation: Option<&'static str>,
}

impl Package {
    pub fn len(&self) -> usize {
        self.pulses.len()
    }

    /// Rejoin marks split by a dropout too short to be a symbol.
    ///
    /// A weak burst does not fail cleanly. Its envelope crosses back under the
    /// threshold for a few microseconds in the middle of a mark, and what was
    /// one symbol arrives as two marks with a sliver of gap between them. Every
    /// timing measured after that is wrong, so a capture that is 20 dB above
    /// the point where the bits stop being recoverable can still decode
    /// nothing at all.
    ///
    /// The threshold for "too short" is taken from the burst rather than set
    /// in advance, which is Universal Radio Hacker's trick: the shortest
    /// widths that occur *often* are the symbol, so anything well under them is
    /// damage. A constant cannot do this because the symbol width is what
    /// varies between devices, by two orders of magnitude across this corpus.
    ///
    /// Returns how many joins were made.
    pub fn merge_dropouts(&mut self) -> usize {
        let Some(tol) = self.glitch_tolerance_us() else { return 0 };
        let before = self.pulses.len();
        let mut out: Vec<Pulse> = Vec::with_capacity(before);
        for p in self.pulses.iter().copied() {
            match out.last_mut() {
                // The previous pulse's gap was a dropout, not a gap: the two
                // marks and the sliver between them are one mark.
                Some(prev) if prev.gap > 0 && prev.gap < tol => {
                    prev.mark += prev.gap + p.mark;
                    prev.gap = p.gap;
                }
                _ => out.push(p),
            }
        }
        // A tolerance that swallows most of the burst was the wrong estimate,
        // and half a burst rewritten is worse than a burst left alone.
        if out.len() * 2 < before {
            return 0;
        }
        self.pulses = out;
        before - self.pulses.len()
    }

    /// Width below which a gap is damage rather than a symbol.
    ///
    /// An eighth of the median gap. The median is what the transmitter is
    /// doing: a burst coming apart grows short fragments at both ends of the
    /// distribution but its middle stays where it was, so the middle is the
    /// only stable thing to measure against. The shortest width is not, which
    /// is the trap: as a burst fragments, the shortest width is itself damage,
    /// so a tolerance derived from it shrinks exactly when it needs to grow.
    ///
    /// An eighth is well under the ratio between the two symbols of every
    /// coding here, which is two to one at its narrowest, so a real short
    /// symbol is never mistaken for a dropout.
    fn glitch_tolerance_us(&self) -> Option<u32> {
        if self.pulses.len() < 3 {
            return None;
        }
        // The final gap is the timeout that ended the burst rather than a gap
        // the transmitter sent, so it is left out of the estimate.
        let mut gaps: Vec<u32> =
            self.pulses[..self.pulses.len() - 1].iter().map(|p| p.gap).collect();
        gaps.retain(|g| *g > 0);
        if gaps.len() < 2 {
            return None;
        }
        gaps.sort_unstable();
        Some((gaps[gaps.len() / 2] / 8).max(1))
    }

    pub fn is_empty(&self) -> bool {
        self.pulses.is_empty()
    }

    /// Total on-air duration in microseconds, excluding the trailing timeout.
    pub fn duration_us(&self) -> u64 {
        self.pulses.iter().map(|p| p.mark as u64 + p.gap as u64).sum::<u64>()
            - self.pulses.last().map(|p| p.gap as u64).unwrap_or(0)
    }

    /// Histogram of mark widths, bucketed to `tol_us`.
    ///
    /// Reading this is how an unknown protocol gets identified by hand: PWM
    /// shows two clear mark clusters, PPM shows one mark cluster and two gap
    /// clusters, Manchester shows clusters at T and 2T in both.
    pub fn mark_histogram(&self, tol_us: u32) -> Vec<(u32, usize)> {
        histogram(self.pulses.iter().map(|p| p.mark), tol_us)
    }

    pub fn gap_histogram(&self, tol_us: u32) -> Vec<(u32, usize)> {
        // The trailing gap is a timeout, not signal, so leave it out.
        let n = self.pulses.len().saturating_sub(1);
        histogram(self.pulses[..n].iter().map(|p| p.gap), tol_us)
    }
}

fn histogram(vals: impl Iterator<Item = u32>, tol_us: u32) -> Vec<(u32, usize)> {
    let mut buckets: Vec<(u32, usize, u64)> = Vec::new();
    for v in vals {
        match buckets.iter_mut().find(|(c, _, _)| v.abs_diff(*c) <= tol_us) {
            Some((c, n, sum)) => {
                *n += 1;
                *sum += v as u64;
                *c = (*sum / *n as u64) as u32;
            }
            None => buckets.push((v, 1, v as u64)),
        }
    }
    buckets.sort_by_key(|(c, _, _)| *c);
    buckets.into_iter().map(|(c, n, _)| (c, n)).collect()
}

/// One packet on the bus: what a demodulator produced, and where.
///
/// The common currency between everything that produces packets and
/// everything that consumes them. A channel bank produces timings, a Mode S
/// demodulator produces frames, and a log, a list, a map or a tracker take
/// either without caring which front end was involved.
///
/// What is *not* here is a parse. A model name and a field map are
/// conclusions, and a conclusion travelling in place of its evidence cannot
/// be checked, corrected or re-decoded later.
#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    /// Wall clock when the block carrying it was processed, in microseconds
    /// since the epoch.
    pub at_us: u64,
    /// Where it was received, in Hz: the channel's own centre in a bank.
    pub center_hz: u64,
    /// The width it was heard through. The same burst read through a 31 kHz
    /// channel and a 125 kHz one is not the same recording.
    pub bandwidth_hz: u32,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    /// What the burst was measured to be keyed on. See
    /// [`Package::modulation`].
    pub modulation: Option<&'static str>,
    pub body: PacketBody,
    /// The burst itself, as complex samples at the rate it was read at,
    /// when the front end kept them.
    ///
    /// Shared rather than copied, since the packages of one burst all refer
    /// to the same samples, and carried in memory only: a log keeps
    /// timings and measurements, a list keeps the samples of what it shows.
    /// They are what an unknown device is worked out from, the way Universal
    /// Radio Hacker shows a burst beside its bits.
    pub iq: Option<std::sync::Arc<IqBurst>>,
    /// Speech the demodulator decoded, for a protocol that carries voice.
    ///
    /// The payload of a voice transmission is what was said, and no rendering
    /// of its bytes is that. Shared for the same reason the samples are: a
    /// minute of speech is megabytes, and the log, the player and anything
    /// writing it to a file all want the one copy.
    pub audio: Option<std::sync::Arc<Speech>>,
    /// What the burst was measured to be, when something measured it before
    /// deciding how to read it. Travels with the timings because it is
    /// evidence about the same burst: a chirp's sweep rate or a keyed
    /// signal's tone separation is what identifies a device the tables do
    /// not know, and a burst no front end reads has nothing else to say.
    pub measure: Option<Measure>,
}

/// A burst's samples, as the front end that read it saw them: the lead-in,
/// the burst and the silence that ended it.
#[derive(Clone, Debug, PartialEq)]
pub struct IqBurst {
    /// Sample rate of `samples`.
    pub rate: f64,
    /// RF centre the samples are at baseband from.
    pub center_hz: u64,
    pub samples: Vec<C32>,
}

/// What a burst was measured to be, before any decoder read it.
#[derive(Clone, Debug, PartialEq)]
pub struct Measure {
    /// The classifier's verdict, as its label: "OOK", "2-FSK", "chirp",
    /// "carrier", "unknown".
    pub modulation: &'static str,
    /// How far that verdict stood above the runner-up, 0 to 1.
    pub confidence: f32,
    /// Which front end the burst was sent to, or "none".
    pub front_end: &'static str,
    /// The mode the parameters place it in, when one is known: "LoRa SF9
    /// BW125".
    pub mode: Option<String>,
    pub duration_us: u32,
    pub bandwidth_hz: f32,
    /// Symbol rate in baud, zero when none was found.
    pub baud: f32,
    /// Tone separation in hertz, for keyed signals; zero otherwise.
    pub separation_hz: f32,
    /// Sweep rate in hertz per second, for chirps; zero otherwise.
    pub sweep_hz_s: f32,
    /// Period of a repeating structure in microseconds, a cyclic prefix or
    /// a chip sequence; zero otherwise.
    pub symbol_period_us: f32,
}

/// The classifier's labels, so a label read back from a file is the same
/// static string the classifier uses.
pub const MODULATION_LABELS: [&str; 13] = [
    "OOK", "ASK", "2-FSK", "4-FSK", "MSK", "BPSK", "QPSK", "chirp", "OFDM", "DSSS",
    "noise-like", "carrier", "unknown",
];

/// The front ends a burst can be sent to.
pub const FRONT_ENDS: [&str; 6] = ["ook", "ask", "fsk", "c4fm", "ook+fsk", "none"];

impl Measure {
    /// The label as one of [`MODULATION_LABELS`], or "unknown".
    pub fn label(s: &str) -> &'static str {
        MODULATION_LABELS.iter().copied().find(|l| *l == s).unwrap_or("unknown")
    }

    /// The front end as one of [`FRONT_ENDS`], or "none".
    pub fn front(s: &str) -> &'static str {
        FRONT_ENDS.iter().copied().find(|l| *l == s).unwrap_or("none")
    }

    /// One line a list can show: what it was, how sure, and the numbers
    /// that identify it.
    ///
    /// Only the numbers that belong to the verdict: the tone separation of
    /// a keyed signal, the sweep of a chirp, the period of a multi-carrier
    /// or spread signal. Every feature is measured on every burst, and the
    /// ones that belong to the hypotheses that lost are noise here.
    pub fn summary(&self) -> String {
        let mut parts = vec![format!("{} {:.2}", self.modulation, self.confidence)];
        if let Some(m) = &self.mode {
            parts.push(m.clone());
        }
        if self.bandwidth_hz > 0.0 {
            parts.push(format!("{:.1} kHz wide", self.bandwidth_hz / 1e3));
        }
        let keyed = matches!(self.modulation, "OOK" | "ASK" | "2-FSK" | "4-FSK" | "MSK" | "BPSK" | "QPSK");
        if keyed && self.baud > 0.0 {
            parts.push(format!("{:.0} baud", self.baud));
        }
        if matches!(self.modulation, "2-FSK" | "4-FSK" | "MSK") && self.separation_hz > 0.0 {
            parts.push(format!("tones {:.1} kHz apart", self.separation_hz / 1e3));
        }
        if self.modulation == "chirp" && self.sweep_hz_s.abs() > 0.0 {
            parts.push(format!("sweep {:.1} MHz/s", self.sweep_hz_s / 1e6));
        }
        if matches!(self.modulation, "OFDM" | "DSSS") && self.symbol_period_us > 0.0 {
            parts.push(format!("period {:.1} us", self.symbol_period_us));
        }
        parts.push(format!("{:.1} ms", self.duration_us as f64 / 1e3));
        parts.join(", ")
    }
}

/// Decoded speech, at whatever rate the codec produces.
#[derive(Clone, Debug, PartialEq)]
pub struct Speech {
    pub pcm: Vec<f32>,
    pub rate: f64,
}

impl Speech {
    pub fn seconds(&self) -> f64 {
        self.pcm.len() as f64 / self.rate.max(1.0)
    }
}

/// What the demodulator actually produced.
#[derive(Clone, Debug, PartialEq)]
pub enum PacketBody {
    /// A burst, as mark and gap timings.
    Pulses(Vec<Pulse>),
    /// A whole frame from a demodulator that produces bytes.
    Frame(Vec<u8>),
}

impl Packet {
    /// The burst as a package again, ready to hand to a decoder.
    pub fn package(&self) -> Option<Package> {
        match &self.body {
            PacketBody::Pulses(p) => Some(Package {
                pulses: p.clone(),
                snr_db: self.snr_db,
                rssi_dbfs: self.rssi_dbfs,
                start_sample: 0,
                center_hz: self.center_hz,
                modulation: self.modulation,
            }),
            PacketBody::Frame(_) => None,
        }
    }

    pub fn frame(&self) -> Option<&[u8]> {
        match &self.body {
            PacketBody::Frame(b) => Some(b),
            PacketBody::Pulses(_) => None,
        }
    }
}

#[cfg(test)]
mod dropout_tests {
    use super::*;

    fn pkg(pulses: &[(u32, u32)]) -> Package {
        Package {
            pulses: pulses.iter().map(|(m, g)| Pulse { mark: *m, gap: *g }).collect(),
            ..Default::default()
        }
    }

    #[test]
    fn a_dropout_inside_a_mark_is_rejoined() {
        // 500 us marks on a 500 us raster, with one mark split by a 20 us dip.
        let mut p = pkg(&[(500, 500), (240, 20), (240, 500), (500, 500)]);
        assert_eq!(p.merge_dropouts(), 1);
        assert_eq!(
            p.pulses,
            pkg(&[(500, 500), (500, 500), (500, 500)]).pulses,
            "the split mark should read as one 500 us mark again"
        );
    }

    #[test]
    fn a_real_short_symbol_is_left_alone() {
        // PWM: 250 and 500 us marks, 250 us gaps. Nothing here is damage, and
        // a tolerance that ate the short symbol would destroy every bit.
        let mut p = pkg(&[(250, 250), (500, 250), (250, 250), (500, 250)]);
        assert_eq!(p.merge_dropouts(), 0);
        assert_eq!(p.pulses.len(), 4);
    }

    #[test]
    fn one_odd_reading_does_not_set_the_tolerance() {
        // A single 8 us sliver must not make 8 us the yardstick: the estimate
        // comes from the shortest width that repeats.
        let mut p = pkg(&[(8, 500), (500, 500), (500, 500), (500, 500)]);
        p.merge_dropouts();
        assert!(p.pulses.len() >= 3);
    }

    #[test]
    fn a_burst_that_would_be_mostly_rewritten_is_left_alone() {
        // Every gap under the tolerance means the estimate was wrong, and
        // collapsing the burst to one pulse invents a signal that was not sent.
        let mut p = pkg(&[(100, 5), (100, 5), (100, 5), (100, 5), (100, 5)]);
        let before = p.pulses.clone();
        p.merge_dropouts();
        assert_eq!(p.pulses, before);
    }
}
