//! Wireless M-Bus as a stage: a source's stream in, meter frames out.
//!
//! One demodulator over one source, as the pager and packet stages are.
//! It reads complex baseband at whatever rate the source was cut out at,
//! provided that is four samples a chip or more, and puts out each frame
//! that passed its CRCs as bytes, from the length field on. The frames
//! reach the packet bus as any other frame does, and the protocols node
//! reads the address out of them; see [`decode::wmbus`].

use common::Result;
use dsp::wmbus::{Demod, CHIP_RATE};
use pipeline::event::{Decoded, Event};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// Width a mode T or C transmission occupies, for the port and the log.
pub const CHANNEL_WIDTH_HZ: f64 = 250_000.0;

pub struct WmbusNode {
    demod: Option<Demod>,
    frames: u64,
}

impl Default for WmbusNode {
    fn default() -> Self {
        Self::new()
    }
}

impl WmbusNode {
    pub fn new() -> Self {
        Self { demod: None, frames: 0 }
    }

    /// Frames that passed their CRCs since the node was made.
    pub fn frames(&self) -> u64 {
        self.frames
    }
}

impl Simple for WmbusNode {
    fn name(&self) -> &str {
        "wmbus"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("wmbus reads complex baseband"));
        }
        let d = Demod::new(i.spec.rate);
        if !d.usable() {
            return Err(common::Error::other(format!(
                "wmbus needs at least {} S/s: its chips are {} us wide",
                4.0 * CHIP_RATE,
                1e6 / CHIP_RATE
            )));
        }
        self.demod = Some(d);
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(d)) = (i.as_iq(), self.demod.as_mut()) else { return Ok(()) };
        let rate = c.inputs[0].spec.rate.max(1.0);
        for f in d.process(iq) {
            self.frames += 1;
            c.emit(Event::Metric { name: "wmbus_mode", value: if f.mode == dsp::wmbus::Mode::T { 1.0 } else { 2.0 } });
            let _ = rate;
            o.frames_mut().push(f.bytes.clone());
        }
        Ok(())
    }

    fn reset(&mut self) {
        if let Some(d) = &mut self.demod {
            d.reset();
        }
    }
}

/// What the protocols node makes of a meter frame: who sent it and what it
/// is, with the bytes as they arrived.
pub fn wmbus_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = decode::wmbus::parse(bytes, None)?;
    let mut d = Decoded::bytes("Wireless-MBus", center, 0.0, bytes.to_vec())
        .with_modulation("2-FSK")
        .with_crc(Some(true))
        .with_bandwidth(CHANNEL_WIDTH_HZ);
    let fields: Vec<(String, common::Value)> =
        r.fields.iter().filter(|(k, _)| k.as_str() != "data").map(|(k, v)| (k.clone(), v.clone())).collect();
    let m = r.get("M").map(|v| v.to_string()).unwrap_or_default();
    let id = r.get("id").map(|v| v.to_string()).unwrap_or_default();
    let kind = r.get("type_string").map(|v| v.to_string()).unwrap_or_default();
    let enc = r.get("payload_encrypted").is_some();
    let text = format!(
        "{m} {kind} {id}{}",
        if enc { ", payload encrypted" } else { "" }
    );
    d = d.with_text(text.clone()).with_detail(r.fields_line()).with_fields(fields);
    Some(d)
}
