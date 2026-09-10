//! Channels the receiver has decided to keep listening on.

use common::SourceId;
use dsp::Owned;
use pipeline::event::Event;
use pipeline::registry::Settings;

use super::AutoNode;
use crate::protocol::{self, Stickiness, CHANNEL_WIDTH_TOLERANCE};

/// A channel a front end has read something on, kept for as long as the
/// node runs.
///
/// Detection finds what is transmitting; it does not know what will. A
/// channel that has produced a decoded frame is one that will produce
/// another, on the same frequency, in the same width, read by the same
/// front end, and from then on it does not have to be found again: it is
/// cut out continuously and that front end alone reads it. The cases this
/// pays for are the ones detection gets wrong on a channel it has already
/// proved: a burst too strong for the converter, which lights the whole
/// span and measures as two megahertz wide, and a weak one that flickers
/// around the open threshold and comes out as three slivers. Where a
/// transmitter moves, the front end that reads it is the one to follow it;
/// the channel here is where it was heard.
///
/// Not saved. A channel is remembered for a session, and a session that
/// starts fresh finds its channels the same way it did the first time.
pub(super) struct Sticky {
    pub(super) id: SourceId,
    pub(super) name: &'static str,
    pub(super) center_hz: f64,
    pub(super) width_hz: f64,
    /// How long the channel is kept after the last decode on it, or for
    /// the session.
    pub(super) hold_s: Option<f64>,
    /// When something last decoded there, in seconds of stream.
    pub(super) last_heard_s: f64,
    /// The channel that asked for this one, as (protocol, centre), for a
    /// side channel that goes when its parent does.
    pub(super) parent: Option<(&'static str, f64)>,
    /// What the asker said the decoder there should be told.
    pub(super) settings: Settings,
}

/// Source ids counted down from the top, where the detector's never reach.
pub(super) const STICKY_ID_BASE: u64 = u64::MAX - 1_000_000;

/// Every channel the node is keeping, and the clock their holds are
/// measured on.
///
/// One owner for what was three names for the same thing: the channels
/// themselves, the ones waiting to be cut out, and the ones waiting to be
/// given back. Each of those was written in one file and read in another,
/// so a channel could be forgotten and still opened on the next block.
#[derive(Default)]
pub(super) struct Memory {
    channels: Vec<Sticky>,
    /// Channels ever remembered, so an id is never reused after a channel
    /// is forgotten.
    made: u64,
    /// Channels to cut out from the next block on: newly heard ones, and
    /// after a rebuild every one the span still covers.
    pending: Vec<SourceId>,
    /// Channels to stop cutting out on the next block.
    expiring: Vec<SourceId>,
    /// Seconds of stream so far, the clock a hold is measured on.
    now_s: f64,
}

impl Memory {
    pub(super) fn channels(&self) -> &[Sticky] {
        &self.channels
    }

    pub(super) fn find(&self, id: SourceId) -> Option<&Sticky> {
        self.channels.iter().find(|s| s.id == id)
    }

    pub(super) fn set_now(&mut self, now_s: f64) {
        self.now_s = now_s;
    }

    /// Channels to start cutting out, in the order they were remembered.
    /// One still busy is handed back with [`Memory::wait_for`].
    pub(super) fn take_pending(&mut self) -> Vec<SourceId> {
        std::mem::take(&mut self.pending)
    }

    pub(super) fn wait_for(&mut self, id: SourceId) {
        self.pending.push(id);
    }

    pub(super) fn take_expiring(&mut self) -> Vec<SourceId> {
        std::mem::take(&mut self.expiring)
    }

    /// Every channel a front end is reading, as the detector is told them.
    pub(super) fn owned(&self, center_hz: f64) -> impl Iterator<Item = Owned> + '_ {
        self.channels.iter().map(move |st| Owned {
            lo_hz: st.center_hz - st.width_hz / 2.0 - center_hz,
            hi_hz: st.center_hz + st.width_hz / 2.0 - center_hz,
            max_width_hz: st.width_hz * CHANNEL_WIDTH_TOLERANCE,
        })
    }

    /// Say something decoded on whatever channel covers `hz`, which is what
    /// keeps it from being given back.
    pub(super) fn heard_at(&mut self, hz: f64) {
        let now = self.now_s;
        for st in self.channels.iter_mut() {
            if (st.center_hz - hz).abs() <= st.width_hz / 2.0 {
                st.last_heard_s = now;
            }
        }
    }

    /// Channels nothing has decoded on for their hold, dropped and queued to
    /// be given back to the detector.
    pub(super) fn expire(&mut self) -> Vec<SourceId> {
        let now = self.now_s;
        let out: Vec<SourceId> = self
            .channels
            .iter()
            .filter(|s| s.hold_s.is_some_and(|h| now - s.last_heard_s > h))
            .map(|s| s.id)
            .collect();
        if !out.is_empty() {
            self.expiring.extend(out.iter().copied());
            self.forget(&out);
        }
        out
    }

    /// Drop remembered channels, and every channel they asked for.
    pub(super) fn forget(&mut self, ids: &[SourceId]) {
        let gone: Vec<(&'static str, f64)> = self
            .channels
            .iter()
            .filter(|s| ids.contains(&s.id))
            .map(|s| (s.name, s.center_hz))
            .collect();
        if gone.is_empty() {
            return;
        }
        let mut children: Vec<SourceId> = Vec::new();
        self.channels.retain(|s| {
            if ids.contains(&s.id) {
                return false;
            }
            if s.parent.is_some_and(|p| gone.contains(&p)) {
                children.push(s.id);
                return false;
            }
            true
        });
        self.pending
            .retain(|id| !ids.contains(id) && !children.contains(id));
        self.expiring.extend(children.iter().copied());
        if !children.is_empty() {
            self.forget(&children);
        }
    }

    /// Keep a channel, unless it is one already kept, and have it cut out
    /// from the next block on.
    pub(super) fn remember(
        &mut self,
        name: &'static str,
        center_hz: f64,
        width_hz: f64,
        hold_s: Option<f64>,
        parent: Option<(&'static str, f64)>,
        settings: Settings,
    ) -> Option<Event> {
        if width_hz <= 0.0 {
            return None;
        }
        let same =
            |s: &Sticky| s.name == name && (s.center_hz - center_hz).abs() <= s.width_hz / 2.0;
        if self.channels.iter().any(same) {
            return None;
        }
        let id = SourceId(STICKY_ID_BASE + self.made);
        self.made += 1;
        self.channels.push(Sticky {
            id,
            name,
            center_hz,
            width_hz,
            hold_s,
            last_heard_s: self.now_s,
            parent,
            settings,
        });
        self.pending.push(id);
        Some(Event::Warning {
            message: format!(
                "{name} read {:.4} MHz; keeping that channel open for the session",
                center_hz / 1e6
            ),
        })
    }

    /// Cut every channel out again, from wherever the stream is now: what a
    /// rebuild leaves to be done, since the span it was cut from has gone.
    pub(super) fn cut_again(&mut self) {
        self.pending = self.channels.iter().map(|s| s.id).collect();
    }
}

impl AutoNode {
    /// Every channel a front end owns: the ones remembered because
    /// something decoded there, and the span-wide fronts, which own the
    /// frequency the standard put them on from the moment the span reaches
    /// it. Both are places the receiver has decided to listen, both are
    /// closed to the detector, and the spectrum draws them the same way.
    pub fn locked_channels(&self) -> Vec<(&'static str, f64, f64)> {
        let mut out: Vec<(&'static str, f64, f64)> = self
            .wide
            .iter()
            .filter_map(|m| m.band.map(|(lo, hi)| (m.name, (lo + hi) / 2.0, hi - lo)))
            .collect();
        out.extend(self.remembered());
        out
    }

    /// Channels front ends have read something on this session, as
    /// (front end, centre, width) in hertz.
    pub fn remembered(&self) -> Vec<(&'static str, f64, f64)> {
        self.memory
            .channels()
            .iter()
            .map(|s| (s.name, s.center_hz, s.width_hz))
            .collect()
    }

    /// Hand the detector every channel a front end owns, so nothing opens
    /// inside one. The span-wide fronts own theirs from the moment the span
    /// reaches them; a remembered channel from the moment something decoded
    /// on it.
    ///
    /// A band a span-wide front end owns is closed to whatever is in it. A
    /// remembered channel is closed to what the front end there is already
    /// reading, and open to a signal too wide to be that: the detector is the
    /// only thing that can notice a second transmitter sharing the frequency,
    /// since nothing else is looking there any more.
    pub(super) fn apply_locked(&mut self) {
        let c = self.center.as_f64();
        let mut owned: Vec<Owned> = self
            .wide
            .iter()
            .filter_map(|m| m.band)
            .map(|(lo, hi)| Owned {
                lo_hz: lo - c,
                hi_hz: hi - c,
                max_width_hz: f64::INFINITY,
            })
            .collect();
        owned.extend(self.memory.owned(c));
        self.watch.set_owned(owned);
    }

    /// Drop remembered channels, and every channel they asked for.
    pub(super) fn forget(&mut self, ids: &[SourceId]) {
        self.memory.forget(ids);
        self.apply_locked();
    }

    /// Remember a channel a front end has just read, for as long as its
    /// protocol says.
    pub(super) fn remember(
        &mut self,
        name: &'static str,
        center_hz: f64,
        width_hz: f64,
    ) -> Option<Event> {
        let hold_s = match protocol::by_id(name).map(|p| p.stickiness()) {
            Some(Stickiness::Forget) => return None,
            Some(Stickiness::Latch { hold_s }) => hold_s,
            Some(Stickiness::Claim) | None => None,
        };
        self.remember_for(name, center_hz, width_hz, hold_s, None, Settings::new())
    }

    pub(super) fn remember_for(
        &mut self,
        name: &'static str,
        center_hz: f64,
        width_hz: f64,
        hold_s: Option<f64>,
        parent: Option<(&'static str, f64)>,
        settings: Settings,
    ) -> Option<Event> {
        let e = self
            .memory
            .remember(name, center_hz, width_hz, hold_s, parent, settings)?;
        self.apply_locked();
        Some(e)
    }
}
