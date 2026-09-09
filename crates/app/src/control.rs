//! Control links: where the sticks are on every handset in earshot.
//!
//! A row is one transmitter, not one packet. A link sends a frame every few
//! milliseconds and reading a list at that rate is impossible, so what a
//! person wants is the state: this handset, these sixteen channels, armed or
//! not, last heard a moment ago.
//!
//! # A frame carries part of the picture, so the picture is merged
//!
//! No link sends all sixteen channels every frame. FrSky alternates banks,
//! sending 1 to 8 and then 9 to 16; ExpressLRS's ordinary rate sends four
//! sticks and hides the switches in a field this does not unpack, and its
//! Full rates send eight, either the low aux group or the high one. So a
//! frame updates the channels it carried and leaves the rest alone, and a
//! channel nothing has ever sent stays absent rather than showing as a stick
//! at its stop.
//!
//! # Nothing here knows a protocol
//!
//! A row exists because a decode carried [`common::ReportDetail::Control`]
//! and named who transmitted. The microseconds are the decoder's, converted
//! where it knew the scale, so a link added later appears here the day its
//! decoder fills the report in.

use crate::radio::DecodeRecord;
use std::time::{Duration, Instant};

/// How long after its last frame a link is still counted as live. A control
/// link sends tens of frames a second, so a second of silence is already a
/// link that has stopped rather than one between frames.
pub const LIVE: Duration = Duration::from_secs(2);

/// How long a link stays in the list after its last frame. Long enough to
/// walk back to the pane and see what the last positions were.
const FORGET: Duration = Duration::from_secs(5 * 60);

/// Beyond this many links the least recently heard are dropped. There is no
/// band where hundreds of handsets are in earshot, so this is a guard rather
/// than a working limit.
const MAX_LINKS: usize = 64;

/// The lowest and highest servo pulse a bar is drawn against, in
/// microseconds.
///
/// The span every one of these links can send rather than the span a stick
/// reaches: CRSF runs 988 to 2012, FrSky's PXX a little wider, and a channel
/// pushed past its endpoint by a mixer should show as pushed past rather than
/// clipped to the end of the bar.
pub const RANGE_US: (u16, u16) = (900, 2100);

/// One control link and the last of everything it has sent.
#[derive(Clone, Debug)]
pub struct Control {
    /// The protocol, from the decode's own name.
    pub system: String,
    /// The transmitter, as the decoder identified it: an ExpressLRS UID, a
    /// FrSky handset id.
    pub id: String,
    /// Where it was last heard, in hertz. A hopping link moves every frame,
    /// so this is where the last one landed rather than a channel.
    pub channel_hz: f64,
    pub first: Instant,
    pub last: Instant,
    pub frames: u64,
    /// The channels, merged across frames, in microseconds.
    pub channels: [Option<u16>; common::CONTROL_CHANNELS],
    /// When each channel was last carried, so a pane can say a value is old
    /// without dropping it.
    pub channel_at: [Option<Instant>; common::CONTROL_CHANNELS],
    pub armed: Option<bool>,
    pub uplink_power_mw: Option<u16>,
    pub last_rssi_dbfs: f32,
    pub best_rssi_dbfs: f32,
}

impl Control {
    pub fn age(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.last)
    }

    pub fn live(&self, now: Instant) -> bool {
        self.age(now) < LIVE
    }

    /// Frames a second, measured over the whole time it has been heard.
    ///
    /// What tells one ExpressLRS rate from another and says whether a link is
    /// healthy. `None` until there is a span to divide by, since one frame
    /// over no time is not a rate.
    pub fn frame_rate(&self) -> Option<f64> {
        let span = self
            .last
            .saturating_duration_since(self.first)
            .as_secs_f64();
        (span > 0.25).then(|| self.frames as f64 / span)
    }

    /// How many channels have ever been carried, which is the closest thing
    /// to how many the model has.
    pub fn carried(&self) -> usize {
        self.channels.iter().filter(|c| c.is_some()).count()
    }
}

/// Every control link heard, most recently first.
#[derive(Default)]
pub struct Controls {
    seen: Vec<Control>,
}

impl Controls {
    /// Fold one decode in, and say whether it carried sticks at all.
    pub fn update(&mut self, rec: &DecodeRecord, at: Instant) -> bool {
        let common::ReportDetail::Control {
            channels,
            armed,
            uplink_power_mw,
        } = &rec.report
        else {
            return false;
        };
        // Who sent it, or there is no row to put it in: a control report with
        // nobody attached would merge two handsets into one set of sticks.
        let Some(id) = rec
            .identity
            .as_ref()
            .map(|i| i.id.clone())
            .filter(|i| !i.is_empty())
        else {
            return false;
        };
        let system = rec
            .model
            .split('-')
            .next()
            .unwrap_or(&rec.model)
            .to_string();
        let found = self
            .seen
            .iter_mut()
            .find(|c| c.system == system && c.id == id);
        let c = match found {
            Some(c) => c,
            None => {
                self.seen.push(Control {
                    system,
                    id,
                    channel_hz: rec.freq,
                    first: at,
                    last: at,
                    frames: 0,
                    channels: [None; common::CONTROL_CHANNELS],
                    channel_at: [None; common::CONTROL_CHANNELS],
                    armed: None,
                    uplink_power_mw: None,
                    last_rssi_dbfs: rec.rssi_dbfs,
                    best_rssi_dbfs: rec.rssi_dbfs,
                });
                self.seen.last_mut().expect("just pushed")
            }
        };
        c.last = at;
        c.channel_hz = rec.freq;
        c.frames += 1;
        c.last_rssi_dbfs = rec.rssi_dbfs;
        if rec.rssi_dbfs > c.best_rssi_dbfs || c.best_rssi_dbfs.is_nan() {
            c.best_rssi_dbfs = rec.rssi_dbfs;
        }
        // Merged, not replaced: a frame that did not carry a channel says
        // nothing about it, and taking the absence as a position would put
        // half the sticks at zero every other frame.
        for (i, us) in channels.iter().enumerate() {
            if let Some(us) = us {
                c.channels[i] = Some(*us);
                c.channel_at[i] = Some(at);
            }
        }
        if armed.is_some() {
            c.armed = *armed;
        }
        if uplink_power_mw.is_some() {
            c.uplink_power_mw = *uplink_power_mw;
        }
        self.forget(at);
        true
    }

    fn forget(&mut self, now: Instant) {
        self.seen.retain(|c| c.age(now) < FORGET);
        if self.seen.len() > MAX_LINKS {
            self.seen.sort_by(|a, b| b.last.cmp(&a.last));
            self.seen.truncate(MAX_LINKS);
        }
    }

    /// Every link, most recently heard first.
    pub fn active(&self, now: Instant) -> Vec<&Control> {
        let mut v: Vec<&Control> = self.seen.iter().filter(|c| c.age(now) < FORGET).collect();
        v.sort_by(|a, b| b.last.cmp(&a.last));
        v
    }

    pub fn len(&self) -> usize {
        self.seen.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::ReportDetail;

    fn rec(
        model: &str,
        id: &str,
        channels: [Option<u16>; common::CONTROL_CHANNELS],
    ) -> DecodeRecord {
        let mut r = DecodeRecord::for_test(2_415e6, model);
        r.report = ReportDetail::Control {
            channels,
            armed: Some(false),
            uplink_power_mw: None,
        };
        r.identity = Some(common::Identity::new("elrs", id));
        r
    }

    fn bank(base: usize, values: [u16; 8]) -> [Option<u16>; common::CONTROL_CHANNELS] {
        let mut c = [None; common::CONTROL_CHANNELS];
        for (i, v) in values.into_iter().enumerate() {
            c[base + i] = Some(v);
        }
        c
    }

    /// The reason this list exists rather than a filter on the packet list:
    /// a frame carries one bank and the next carries the other, so what a
    /// person sees has to be the two of them merged. Replacing instead would
    /// show half the model's channels as absent every other frame.
    #[test]
    fn two_banks_of_one_handset_are_one_row_with_sixteen_channels() {
        let mut c = Controls::default();
        let now = Instant::now();
        assert!(c.update(&rec("FrSky", "b3fd", bank(0, [1500; 8])), now));
        assert!(c.update(&rec("FrSky", "b3fd", bank(8, [1100; 8])), now));
        assert_eq!(c.len(), 1);
        let row = c.active(now)[0];
        assert_eq!(row.carried(), 16);
        assert_eq!(row.channels[0], Some(1500));
        assert_eq!(row.channels[15], Some(1100));
        assert_eq!(row.frames, 2);
    }

    /// A channel a frame did not carry keeps what it had. ExpressLRS's
    /// ordinary rate sends four sticks and nothing about the switches, so a
    /// link that sent eight once and four since still has eight.
    #[test]
    fn a_channel_a_frame_did_not_carry_keeps_its_last_position() {
        let mut c = Controls::default();
        let now = Instant::now();
        c.update(&rec("ExpressLRS", "6f37", bank(0, [1500; 8])), now);
        let mut four = [None; common::CONTROL_CHANNELS];
        four[..4].copy_from_slice(&[Some(988), Some(988), Some(988), Some(988)]);
        c.update(&rec("ExpressLRS", "6f37", four), now);
        let row = c.active(now)[0];
        assert_eq!(row.channels[0], Some(988));
        assert_eq!(
            row.channels[7],
            Some(1500),
            "the aux channels are still where they were"
        );
        assert_eq!(row.carried(), 8);
    }

    /// Two handsets are two rows, and a decode that carried no sticks is not
    /// a row at all.
    #[test]
    fn a_row_is_a_transmitter_and_a_decode_without_sticks_is_none() {
        let mut c = Controls::default();
        let now = Instant::now();
        c.update(&rec("ExpressLRS", "6f37", bank(0, [1500; 8])), now);
        c.update(&rec("ExpressLRS", "aa01", bank(0, [1500; 8])), now);
        assert_eq!(c.len(), 2);

        let mut sync = DecodeRecord::for_test(2_415e6, "ExpressLRS");
        sync.identity = Some(common::Identity::new("elrs", "6f37"));
        assert!(
            !c.update(&sync, now),
            "a sync packet is not a set of sticks"
        );

        // Nor is a control report from nobody in particular: two handsets
        // would merge into one row of sticks that were never sent together.
        let mut nameless = rec("ExpressLRS", "6f37", bank(0, [1500; 8]));
        nameless.identity = None;
        assert!(!c.update(&nameless, now));
        assert_eq!(c.len(), 2);
    }

    /// A link that has stopped is not a link with its sticks centred, so the
    /// row says how long ago it was heard and stops being live.
    #[test]
    fn a_link_that_stopped_is_no_longer_live() {
        let mut c = Controls::default();
        let now = Instant::now();
        c.update(&rec("ExpressLRS", "6f37", bank(0, [1500; 8])), now);
        let row = c.active(now)[0];
        assert!(row.live(now));
        assert!(!row.live(now + LIVE + Duration::from_millis(1)));
        assert!(
            c.active(now + FORGET).is_empty(),
            "and eventually it is forgotten"
        );
    }

    /// The rate is what tells one ExpressLRS mode from another, and one
    /// frame over no time is not a rate.
    #[test]
    fn the_frame_rate_needs_a_span_to_measure_over() {
        let mut c = Controls::default();
        let now = Instant::now();
        c.update(&rec("ExpressLRS", "6f37", bank(0, [1500; 8])), now);
        assert_eq!(c.active(now)[0].frame_rate(), None);
        for i in 1..=100 {
            c.update(
                &rec("ExpressLRS", "6f37", bank(0, [1500; 8])),
                now + Duration::from_millis(i * 10),
            );
        }
        let rate = c.active(now)[0].frame_rate().expect("a rate");
        assert!((rate - 101.0).abs() < 2.0, "100 Hz measured as {rate}");
    }
}
