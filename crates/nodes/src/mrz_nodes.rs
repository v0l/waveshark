//! Meteo-Radiy MRZ radiosondes as a stage: a source's stream in, gathered
//! records out.
//!
//! 2400 chips a second, Manchester coded, so the data rate is 1200 bits a
//! second. The frame is found by its own first three bytes rather than by a
//! preamble: an MRZ leads with a run of `0xAA`, which is the same alternating
//! pattern as its preamble, so `AA BF 35` is what says where the frame
//! starts and a preamble correlation says only that one is coming.
//!
//! The frame is [`decode::mrz`], the waveform [`dsp::fsk::BitSync`], and the
//! layout is from zilog80's `rs1729/RS`, `demod/mod/mp3h1mod.c`.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::mrz;
pub use decode::mrz::read;
use dsp::fsk::BitSync;
use identify::Signal;
pub use identify::mrz::BAND;
pub use identify::mrz::BAUD;
pub use identify::mrz::CHANNEL_WIDTH_HZ;
pub use identify::mrz::Mrz;
pub use identify::mrz::OCCUPIED_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct MrzNode {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    framer: mrz::Framer,
    frames: u64,
}

impl Default for MrzNode {
    fn default() -> Self {
        Self::new()
    }
}

impl MrzNode {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6).keyed_as(common::Modulation::Fsk2),
            framer: mrz::Framer::new(),
            frames: 0,
        }
    }

    /// Frames whose check held.
    pub fn frames(&self) -> u64 {
        self.frames
    }
}

impl Simple for MrzNode {
    fn name(&self) -> &str {
        "mrz"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("mrz reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "mrz needs at least {} S/s for its {BAUD} chips a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 1.0)
            .keyed_as(common::Modulation::Fsk2);
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(s)) = (i.as_iq(), self.sync.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        s.process(iq, self.framer.sink());
        for record in self.framer.take() {
            o.packets_mut().push(self.meter.packet_now(record));
        }
        self.framer.trim();
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.framer.reset();
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

impl Protocol for Mrz {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz)
            || !matches!(bytes.len(), mrz::RECORD_ECEF | mrz::RECORD_LATLON)
        {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("mrz")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "mrz",
    summary: "Meteo-Radiy MRZ radiosonde frames, 2400 baud Manchester FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(MrzNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bytes as chips: a one is a fall, a zero a rise.
    fn keyed(frame: &[u8]) -> Vec<bool> {
        frame
            .iter()
            .flat_map(|b| (0..8).rev().map(move |k| b >> k & 1 != 0))
            .flat_map(|bit| [bit, !bit])
            .collect()
    }

    fn a_frame() -> Vec<u8> {
        const A: f64 = 6_378_137.0;
        const E2: f64 = 6.694_379_990_141_32e-3;
        let (lat, lon, alt) = (53.35f64, -5.0f64, 4_712.22f64);
        let (sla, cla) = lat.to_radians().sin_cos();
        let (slo, clo) = lon.to_radians().sin_cos();
        let n = A / (1.0 - E2 * sla * sla).sqrt();
        let cm = |v: f64| ((v * 100.0).round() as i32).to_le_bytes();

        let mut f = vec![0u8; mrz::FRAME_ECEF];
        f[..3].copy_from_slice(&mrz::SYNC);
        f[3] = 0x81;
        f[4..7].copy_from_slice(&[5, 42, 20]);
        f[8..12].copy_from_slice(&cm((n + alt) * cla * clo));
        f[12..16].copy_from_slice(&cm((n + alt) * cla * slo));
        f[16..20].copy_from_slice(&cm((n * (1.0 - E2) + alt) * sla));
        f[26] = 11;
        let cs = decode::bits::crc16le(&f[3..48], 0x8005, 0xFFFF).to_le_bytes();
        f[48..50].copy_from_slice(&cs);
        f
    }

    /// The whole chain this file is, on samples and both ways up: the chip
    /// clock, the sync search, the Manchester decode and the frame's own
    /// check.
    #[test]
    fn a_keyed_frame_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 48_000.0;
            let frame = a_frame();
            // A run of 0xAA in front, which is what an MRZ leads with and
            // what the frame's own first byte looks like.
            let mut wire: Vec<bool> = keyed(&[0xAA; 8]);
            wire.extend(keyed(&frame));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(
                &wire.iter().map(|c| c != &inverted).collect::<Vec<_>>(),
                rate,
                BAUD,
                1_200.0,
                0.5,
            );

            let mut n = MrzNode::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                n.sync.as_mut().unwrap().process(block, n.framer.sink());
                got.extend(n.framer.take());
            }
            assert_eq!(got.len(), 1, "{} records, inverted {inverted}", got.len());
            assert_eq!(got[0][..mrz::FRAME_ECEF], frame[..], "not the bytes that were keyed");

            let d = read(&got[0]).expect("a decode");
            let p = d.placed().expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
        }
    }

    /// Twenty seconds of noise produces no frames.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 48_000.0;
        let mut seed = 0x5eed_1234_9876_fedcu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = MrzNode::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut frames = 0;
        for block in iq.chunks(4096) {
            n.sync.as_mut().unwrap().process(block, n.framer.sink());
            frames += n.framer.take().len();
            n.framer.trim();
        }
        assert_eq!(frames, 0, "{frames} frames out of twenty seconds of noise");
    }
}
