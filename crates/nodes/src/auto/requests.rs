//! What the decoders ask for, and what is done about it.

use pipeline::event::{Event, Request};

use super::AutoNode;
use crate::protocol::{self, Stickiness};

impl AutoNode {
    /// Whether what the span-wide decoders have claimed covers the whole
    /// span, since then there is nothing left for the detector to look at.
    ///
    /// A claim is kept until the front end that took it gives it back, not
    /// followed block by block. A picture fades and comes back, a call ends
    /// and the next one starts on the same channel, and a claim that flapped
    /// with the signal would hand the band back to the detector every time
    /// and take it again a moment later. What ends it is a `Release` from
    /// the decoder that claimed, which is how a camera that has left the air
    /// puts the band back.
    pub(super) fn claimed_whole_span(&self) -> bool {
        let (lo, hi) = (
            self.center.as_f64() - self.input_bw / 2.0,
            self.center.as_f64() + self.input_bw / 2.0,
        );
        self.wide
            .iter()
            .filter_map(|m| m.band)
            .any(|(a, b)| a <= lo && hi <= b)
    }

    /// Answer what a decoder asked, and return what only the receiver can
    /// answer.
    ///
    /// `slot` is the source the decoder is on, or None for one over the
    /// span, and `front` is the front end that asked, which the node knows
    /// because it ran it. A claim closes the detector out of the band; a
    /// channel asked for is remembered the way one that decoded is, tied to
    /// the asker; a reshape remembers the wider channel and closes the
    /// source it was cut from, so the remembered one takes over on the next
    /// block; a release drops the decoder and lets the band go. Anything
    /// that needs the dial is handed back.
    pub(super) fn answer(
        &mut self,
        slot: Option<usize>,
        front: &'static str,
        r: Request,
        out: &mut Vec<Event>,
    ) -> Option<Request> {
        let warn = |message: String| Event::Warning { message };
        let half = self.input_bw / 2.0;
        let c0 = self.center.as_f64();
        let in_span = |hz: f64, w: f64| (hz - c0).abs() + w / 2.0 <= half;
        let asker = slot.map(|k| self.slots[k].center_hz.as_f64());
        let name = protocol::by_id(front).map(|p| p.id());
        let hold_of = |name: &str| match protocol::by_id(name).map(|p| p.stickiness()) {
            Some(Stickiness::Latch { hold_s }) => hold_s,
            _ => None,
        };
        match r {
            Request::Claim { lo_hz, hi_hz } => {
                match slot {
                    None => {
                        for m in self.wide.iter_mut().filter(|m| m.name == front) {
                            let (mut lo, mut hi) = (lo_hz, hi_hz);
                            if let Some((a, b)) = m.placed_band {
                                lo = lo.min(a);
                                hi = hi.max(b);
                            }
                            m.band = Some((lo, hi));
                        }
                        self.apply_locked();
                    }
                    Some(_) => {
                        if let Some(name) = name {
                            let (hz, w) = ((lo_hz + hi_hz) / 2.0, hi_hz - lo_hz);
                            if let Some(e) = self.remember(name, hz, w) {
                                out.push(e);
                            }
                        }
                    }
                }
                None
            }
            Request::OpenChannel {
                protocol: p,
                center_hz,
                width_hz,
                role,
                hold_s,
                settings,
            } => {
                let Some(proto) = protocol::by_id(&p) else {
                    out.push(warn(format!(
                        "asked for a channel read by {p:?}, which is not a protocol"
                    )));
                    return None;
                };
                if !in_span(center_hz, width_hz) {
                    return Some(Request::OpenChannel {
                        protocol: p,
                        center_hz,
                        width_hz,
                        role,
                        hold_s,
                        settings,
                    });
                }
                let hold = hold_s.or(hold_of(proto.id()));
                let parent = name.zip(asker);
                if self
                    .remember_for(proto.id(), center_hz, width_hz, hold, parent, settings)
                    .is_some()
                {
                    out.push(warn(format!(
                        "opened a {role} channel at {:.4} MHz for {}",
                        center_hz / 1e6,
                        proto.label()
                    )));
                }
                None
            }
            Request::Reshape { lo_hz, hi_hz } => {
                let (Some(k), Some(name)) = (slot, name) else {
                    return None;
                };
                let (hz, w) = ((lo_hz + hi_hz) / 2.0, hi_hz - lo_hz);
                if !in_span(hz, w) {
                    return Some(Request::Reshape { lo_hz, hi_hz });
                }
                // The stream it was cut from is closed, and the channel it
                // asked for is remembered in its place; if it was itself a
                // remembered channel, that one goes.
                let id = self.slots[k].id;
                self.forget(&[id]);
                self.watch.close_channel(id);
                if let Some(e) = self.remember_for(name, hz, w, hold_of(name), None, Default::default()) {
                    out.push(e);
                }
                None
            }
            Request::Release => {
                let Some(k) = slot else {
                    // A span-wide decoder giving its band back: the detector
                    // and the other span-wide decoders have it again.
                    for m in self.wide.iter_mut().filter(|m| m.name == front) {
                        m.band = None;
                    }
                    self.apply_locked();
                    return None;
                };
                self.slots[k].members.retain(|m| m.name != front);
                if self.slots[k].remembered || self.slots[k].members.is_empty() {
                    let id = self.slots[k].id;
                    self.forget(&[id]);
                    self.watch.close_channel(id);
                }
                None
            }
            Request::Retune { center_hz } => Some(Request::Retune { center_hz }),
        }
    }
}
