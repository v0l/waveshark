//! Meteomodem M10 and M20 radiosondes as a stage: a source's stream in,
//! frames out.
//!
//! Both key 9600-odd chips a second with a transition in every chip pair, so
//! the data rate is half of that and a frame of a hundred bytes takes a
//! sixth of a second. The two differ by 15 baud and by everything above the
//! waveform, so one node reads both and [`decode::m10`] says which it was.
//!
//! The coding is differential rather than Manchester: a bit is whether this
//! chip pair went the same way as the one before it, which is why nothing
//! here has to know which way up the receiver put the signal. What the sync
//! header does is find the frame, not the polarity.
//!
//! The waveform and the header are from zilog80's `rs1729/RS`,
//! `demod/mod/m10m20mod.c`.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::m10;
pub use decode::m10::read;
use dsp::fsk::BitSync;
use identify::Signal;
pub use identify::m10::BAND;
pub use identify::m10::BAUD;
pub use identify::m10::CHANNEL_WIDTH_HZ;
pub use identify::m10::M10;
pub use identify::m10::OCCUPIED_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct M10Node {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    framer: m10::Framer,
}

impl Default for M10Node {
    fn default() -> Self {
        Self::new()
    }
}

impl M10Node {
    pub fn new() -> Self {
        Self { sync: None, meter: crate::FrameMeter::new(1.0, 0, 0.6), framer: m10::Framer::new() }
    }

    /// Frames whose checksum held.
    pub fn frames(&self) -> u64 {
        self.framer.frames()
    }
}

impl Simple for M10Node {
    fn name(&self) -> &str {
        "m10"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("m10 reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "m10 needs at least {} S/s for its {BAUD} chips a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 0.5);
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
        for frame in self.framer.take() {
            o.packets_mut().push(self.meter.packet_now(frame));
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

impl Protocol for M10 {
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
    /// The band is shared with the other sondes, so a frame is claimed on
    /// its own length byte, its type byte and its checksum rather than on
    /// where it was heard.
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || m10::frame_len(bytes).is_none() {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("m10")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "m10",
    summary: "Meteomodem M10 and M20 radiosonde frames, 9600 baud FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(M10Node::new()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use decode::m10::SYNC;

    /// A frame on the air: the sync header, then every bit as a pair of
    /// chips that either repeats the last pair or turns it over.
    fn keyed(frame: &[u8]) -> Vec<bool> {
        let mut chips: Vec<bool> = SYNC.to_vec();
        // The encoder runs on through the header, so the first bit is
        // measured against the header's last pair.
        let mut last = *SYNC.last().unwrap();
        for byte in frame {
            for k in (0..8).rev() {
                let bit = byte >> k & 1 != 0;
                // The bit says whether this pair goes the same way as the
                // one before it, so a one repeats and a zero turns over.
                let pair = last == bit;
                chips.extend([!pair, pair]);
                last = pair;
            }
        }
        chips
    }

    fn an_m10() -> Vec<u8> {
        const B60B60: f64 = (1u32 << 30) as f64 / 90.0;
        let mut f = vec![0u8; 0x65];
        f[0] = 0x64;
        f[1] = 0x9F;
        let put = |f: &mut Vec<u8>, at: usize, b: &[u8]| f[at..at + b.len()].copy_from_slice(b);
        put(&mut f, 0x04, &(1_800i16).to_be_bytes());
        put(&mut f, 0x08, &(1_000i16).to_be_bytes());
        put(&mut f, 0x0A, &(452_540_000u32).to_be_bytes());
        put(&mut f, 0x0E, &((53.35 * B60B60) as i32).to_be_bytes());
        put(&mut f, 0x12, &((-5.0 * B60B60) as i32).to_be_bytes());
        put(&mut f, 0x16, &(4_712_220i32).to_be_bytes());
        put(&mut f, 0x1E, &[11]);
        put(&mut f, 0x20, &(2_357u16).to_be_bytes());
        put(&mut f, 0x5D, &[0x03, 0x00, 0x2A, 0x21, 0x4A]);
        put(&mut f, 0x62, &[57]);
        let cs = decode::m10::check(&f[..0x63]).to_be_bytes();
        put(&mut f, 0x63, &cs);
        f
    }

    /// The whole chain this file is, on samples: the chip clock, the sync
    /// search, the differential decode and the checksum. Run both ways up,
    /// because a differentially coded frame does not care and this proves
    /// it.
    #[test]
    fn a_keyed_frame_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 96_000.0;
            let frame = an_m10();
            let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
            wire.extend(keyed(&frame).into_iter().map(|c| c != inverted));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(&wire, rate, BAUD, 4_800.0, 0.5);

            let mut n = M10Node::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                n.sync.as_mut().unwrap().process(block, n.framer.sink());
                got.extend(n.framer.take());
            }
            assert_eq!(got.len(), 1, "{} frames, inverted {inverted}", got.len());
            assert_eq!(got[0], frame, "the bytes are not the ones that were keyed");

            let d = read(&got[0]).expect("a decode");
            assert_eq!(d.id, "m10");
            let p = d.placed().expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
            assert!(
                d.facts.iter().any(|f| matches!(
                    f,
                    common::packet::Fact::Sensed(r)
                        if r.quantity == common::packet::Quantity::Altitude
                            && (r.value - 4_712.22).abs() < 0.01
                )),
                "{d:?}"
            );
        }
    }

    /// Twenty seconds of noise produces no frames. The sync is 32 chips with
    /// four of slack, so candidates turn up regularly and the checksum is
    /// what refuses them.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 96_000.0;
        let mut seed = 0x1357_9bdf_2468_ace0u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = M10Node::new();
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
