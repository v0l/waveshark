//! Channels the receiver has decided to keep listening on.

use common::SourceId;
use pipeline::event::Event;
use pipeline::registry::Settings;

use super::AutoNode;
use crate::protocol::{self, Stickiness};

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
        self.sticky
            .iter()
            .map(|s| (s.name, s.center_hz, s.width_hz))
            .collect()
    }

    /// Hand the detector every channel a front end owns, so nothing opens
    /// inside one. The span-wide fronts own theirs from the moment the span
    /// reaches them; a remembered channel from the moment something decoded
    /// on it.
    pub(super) fn apply_locked(&mut self) {
        let c = self.center.as_f64();
        let mut ranges: Vec<(f64, f64)> = self.wide.iter().filter_map(|m| m.band).collect();
        ranges.extend(self.sticky.iter().map(|st| {
            (
                st.center_hz - st.width_hz / 2.0,
                st.center_hz + st.width_hz / 2.0,
            )
        }));
        if let Some(d) = self.detector.as_mut() {
            d.set_locked(ranges.iter().map(|(lo, hi)| (lo - c, hi - c)).collect());
        }
    }

    /// Drop remembered channels, and every channel they asked for.
    pub(super) fn forget(&mut self, ids: &[SourceId]) {
        let gone: Vec<(&'static str, f64)> = self
            .sticky
            .iter()
            .filter(|s| ids.contains(&s.id))
            .map(|s| (s.name, s.center_hz))
            .collect();
        if gone.is_empty() {
            return;
        }
        let mut children: Vec<SourceId> = Vec::new();
        self.sticky.retain(|s| {
            if ids.contains(&s.id) {
                return false;
            }
            if s.parent.is_some_and(|p| gone.contains(&p)) {
                children.push(s.id);
                return false;
            }
            true
        });
        self.pending_sticky
            .retain(|id| !ids.contains(id) && !children.contains(id));
        self.expiring.extend(children.iter().copied());
        self.apply_locked();
        if !children.is_empty() {
            self.forget(&children);
        }
    }

    /// Remember a channel a front end has just read, unless it is one
    /// already kept, and have it cut out from the next block on.
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
        if width_hz <= 0.0 {
            return None;
        }
        let same =
            |s: &Sticky| s.name == name && (s.center_hz - center_hz).abs() <= s.width_hz / 2.0;
        if self.sticky.iter().any(same) {
            return None;
        }
        let id = SourceId(STICKY_ID_BASE + self.sticky_made);
        self.sticky_made += 1;
        self.sticky.push(Sticky {
            id,
            name,
            center_hz,
            width_hz,
            hold_s,
            last_heard_s: self.now_s,
            parent,
            settings,
        });
        self.pending_sticky.push(id);
        self.apply_locked();
        Some(Event::Warning {
            stage: self.label.clone(),
            message: format!(
                "{name} read {:.4} MHz; keeping that channel open for the session",
                center_hz / 1e6
            ),
        })
    }
}
