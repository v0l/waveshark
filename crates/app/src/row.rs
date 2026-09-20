//! One reception as the interface holds it, and how it reads.
//!
//! The packet is the whole of what was received; the only thing added here is
//! when this program saw it, because a list ages and orders its rows against
//! the session's own clock and the packet's stamp is the wall clock of the
//! air. Nothing else is copied out of the packet: a row asks it.
//!
//! The words a row shows come from the statements the decoders made, in one
//! renderer, so a protocol that says where it is reads the same on the packet
//! list as it does anywhere else and a decoder gains a readable row by saying
//! what it means rather than by formatting a string.

use std::time::Instant;

use common::Modulation;
use common::packet::{FactKind, Integrity, Packet, Symbols};

/// What a burst nothing claimed is called, everywhere it is asked about.
pub const UNKNOWN: &str = "unknown";

/// A reception, and when the interface heard of it.
#[derive(Clone, Debug)]
pub struct Reception {
    pub at: Instant,
    pub packet: Packet,
}

impl Reception {
    pub fn new(at: Instant, packet: Packet) -> Self {
        Self { at, packet }
    }

    /// What the row is named as, for a person reading it: the protocol that
    /// claimed the burst, or that nothing did.
    pub fn protocol(&self) -> &'static str {
        self.packet.innermost().map(|l| l.id).unwrap_or(UNKNOWN)
    }

    /// Which of the protocol's messages this was.
    pub fn kind(&self) -> &'static str {
        self.packet.innermost().map(|l| l.kind).unwrap_or("")
    }

    /// The system a call, a message or a link on this row belongs to.
    ///
    /// The protocol as its decoder publishes it, which is now the same word
    /// for every message of one system: a table of prefixes used to be needed
    /// here because the protocol name carried the message kind with it.
    pub fn system(&self) -> &'static str {
        self.protocol()
    }

    pub fn freq(&self) -> f64 {
        self.packet.carrier.center_hz as f64
    }

    pub fn channel_hz(&self) -> f64 {
        f64::from(self.packet.carrier.bandwidth_hz)
    }

    pub fn rssi_dbfs(&self) -> f32 {
        self.packet.carrier.rssi_dbfs
    }

    pub fn snr_db(&self) -> f32 {
        self.packet.carrier.snr_db
    }

    pub fn modulation(&self) -> Modulation {
        self.packet.keying.as_ref().map(|k| k.modulation).unwrap_or(Modulation::Unknown)
    }

    pub fn bytes(&self) -> &[u8] {
        self.packet.bytes()
    }

    /// Whether anything stands behind the bytes, in the three states there
    /// are: nothing checked it, it passed, it failed.
    pub fn integrity(&self) -> Integrity {
        self.packet.frame.as_ref().map(|f| f.integrity).unwrap_or(Integrity::Unchecked)
    }

    /// Whether any protocol claimed this burst.
    pub fn is_known(&self) -> bool {
        self.packet.claimed()
    }

    /// Whether somebody wrote what it carries, which is the message view's
    /// entry condition and the decoder's own statement.
    pub fn written(&self) -> bool {
        self.packet.carries().has(FactKind::Message)
    }

    /// What this row says, in words: the statements its decoders made, or
    /// what the burst was measured to be where nothing read it.
    pub fn detail(&self) -> String {
        let said: Vec<String> = self.packet.facts().map(|(_, f)| f.says()).collect();
        if !said.is_empty() {
            return said.join(", ");
        }
        match self.packet.claimed() {
            true => self.kind().replace('_', " "),
            false => keying(&self.packet),
        }
    }

    /// The column headings [`Self::line`] prints under.
    pub fn line_header() -> String {
        format!(
            "{:>8}  {:>13}  {:<10} {:>6} {:>5}  {:<22} {:>3}  info",
            "time", "frequency", "mod", "rssi", "snr", "protocol", "len"
        )
    }

    /// One line in the packet list's columns, timed from `since`.
    pub fn line(&self, since: Instant) -> String {
        format!(
            "{:>8.3}  {:>9.4} MHz  {:<10} {:>6.1} {:>5.1}  {:<22} {:>3}  {}",
            self.at.saturating_duration_since(since).as_secs_f64(),
            self.freq() / 1e6,
            self.modulation(),
            self.rssi_dbfs(),
            self.snr_db(),
            self.protocol(),
            self.bytes().len(),
            self.detail()
        )
    }
}

/// What a burst was measured to be, for a row nothing decoded.
///
/// The verdict and the numbers that belong to it: the tone separation of a
/// keyed signal, the sweep of a chirp, the period of a spread one. Every
/// feature is measured on every burst, and the ones belonging to hypotheses
/// that lost are noise here.
pub fn keying(p: &Packet) -> String {
    let Some(k) = p.keying.as_ref() else { return String::new() };
    let mut parts = vec![k.modulation.to_string()];
    if let common::packet::Knowledge::Measured { confidence } = k.how {
        parts[0] = format!("{} {confidence:.2}", k.modulation);
    }
    let f = &k.params;
    if f.bandwidth_hz > 0.0 {
        parts.push(format!("{:.1} kHz wide", f.bandwidth_hz / 1e3));
    }
    use Modulation as M;
    let keyed = matches!(
        k.modulation,
        M::Ook | M::Ask | M::Fsk2 | M::Fsk4 | M::Msk | M::Psk2 | M::Psk4 | M::Dqpsk
    );
    if keyed && f.baud > 0.0 {
        parts.push(format!("{:.0} baud", f.baud));
    }
    if matches!(k.modulation, M::Fsk2 | M::Fsk4 | M::Msk) && f.separation_hz > 0.0 {
        parts.push(format!("tones {:.1} kHz apart", f.separation_hz / 1e3));
    }
    if k.modulation == M::Chirp && f.sweep_hz_s.abs() > 0.0 {
        parts.push(format!("sweep {:.1} MHz/s", f.sweep_hz_s / 1e6));
    }
    if matches!(k.modulation, M::Ofdm | M::Dsss) && f.symbol_period_us > 0.0 {
        parts.push(format!("period {:.1} us", f.symbol_period_us));
    }
    if let Symbols::Pulses(v) = &k.symbols {
        parts.push(format!("{} pulses", v.len()));
    }
    if p.carrier.duration_us > 0 {
        parts.push(format!("{:.1} ms", p.carrier.duration_us as f64 / 1e3));
    }
    parts.join(", ")
}

/// Receptions built by hand, for a test that needs a row rather than a radio.
#[cfg(test)]
impl Reception {
    /// One decode on a channel, heard now at a workable level.
    pub fn for_test(freq: f64, id: &'static str) -> Self {
        Self::new(
            Instant::now(),
            Packet::heard(common::packet::Carrier::heard(
                common::packet::now_us(),
                freq as u64,
                12_500,
                -40.0,
                20.0,
                common::SourceId(0),
            ))
            .decoded(common::packet::Proto::new(id, "")),
        )
    }

    /// What the innermost decoder states.
    pub fn stating(mut self, f: common::packet::Fact) -> Self {
        if let Some(l) = self.packet.stack.last_mut() {
            l.facts.push(f);
        }
        self
    }

    /// Who it was between, as the decoder named them.
    pub fn linked(mut self, l: common::packet::Link) -> Self {
        if let Some(q) = self.packet.stack.last_mut() {
            q.link = l;
        }
        self
    }

    /// Who was transmitting, as the survey and the control view key on.
    pub fn by(mut self, e: common::packet::Entity) -> Self {
        if let Some(l) = self.packet.stack.last_mut() {
            l.subject = Some(e);
        }
        self
    }

    /// Which of the protocol's messages this was.
    pub fn of_kind(mut self, kind: &'static str) -> Self {
        if let Some(l) = self.packet.stack.last_mut() {
            l.kind = kind;
        }
        self
    }

    /// The bytes it was read from.
    pub fn of_bytes(mut self, bytes: Vec<u8>) -> Self {
        self.packet.frame = Some(common::packet::Frame::of(bytes).checked(Integrity::Passed));
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::packet::{Carrier, Fact, Named, Proto, Quantity, ThingKind};

    fn packet() -> Packet {
        Packet::heard(Carrier::heard(1, 433_920_000, 31_250, -20.0, 15.0, common::SourceId(0)))
    }

    /// A row reads what its decoders said, in the order they said it, and a
    /// burst nothing claimed reads as what it was measured to be.
    #[test]
    fn a_row_says_what_was_decoded() {
        let p = packet().decoded(
            Proto::new("ism", "Fineoffset-WHx080")
                .saying(Fact::Named(Named::new("WH1080", ThingKind::Sensor)))
                .saying(Fact::sensed(Quantity::Temperature, 16.2, common::Unit::Celsius)),
        );
        let r = Reception::new(Instant::now(), p);
        assert_eq!(r.protocol(), "ism");
        assert_eq!(r.detail(), "WH1080, temperature 16.2 \u{b0}C");
        assert!(r.is_known());
        assert!(!r.written());
    }

    #[test]
    fn a_burst_nothing_read_says_what_it_was_measured_to_be() {
        let p = packet().keyed(common::packet::Keying::measured(
            Modulation::Chirp,
            0.82,
            common::packet::KeyingParams {
                bandwidth_hz: 125_000.0,
                sweep_hz_s: 31_250_000.0,
                ..Default::default()
            },
        ));
        let r = Reception::new(Instant::now(), p);
        assert_eq!(r.protocol(), UNKNOWN);
        assert_eq!(r.detail(), "chirp 0.82, 125.0 kHz wide, sweep 31.2 MHz/s");
    }
}
