//! What is on each channel, how loud, and how crowded.
//!
//! A consumer of the packet bus like the survey and the band walk, and for
//! the same reason: which channels are busy is a question about everything
//! heard rather than about one decoder. It reads [`common::packet::Channel`] and
//! nothing else, so a protocol joins this view by saying which channel of
//! which plan it was on rather than by being named here.
//!
//! Two numbers per transmitter, because they disagree. A frame is heard on
//! the channel the tuner is parked on, and an access point on channel 6 is
//! heard on channel 1 through the skirt of a 20 MHz filter. The channel the
//! beacon claims is the one it is working, and the crowding on a channel is
//! computed from what is claimed: a 20 MHz network sits on the two channels
//! either side of its own, which is what makes 1, 6 and 11 the only three
//! that do not overlap.

use common::packet::{Channel, Packet};
use common::{ChannelPlan, Result, Secrecy};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// How many transmitters are kept before the least recently heard is dropped.
///
/// A city street turns up a few hundred access points; the cap is here so a
/// receiver left running overnight cannot grow without bound, and is well
/// clear of what a busy band produces.
const MAX_STATIONS: usize = 1_024;

/// One transmitter, as a channel view lists it.
#[derive(Clone, Debug, PartialEq)]
pub struct Station {
    pub plan: ChannelPlan,
    /// Whatever named it: a MAC address, a BLE address.
    pub id: String,
    /// What it called itself: an SSID, a device name.
    pub name: Option<String>,
    pub vendor: Option<String>,
    /// The channel it is working: what it claims, or where it was heard when
    /// it claims nothing.
    pub channel: u16,
    /// The channel the receiver was on when it was heard, which is the same
    /// number only when the dial was on its channel.
    pub heard_on: u16,
    pub width_hz: u32,
    pub secrecy: Secrecy,
    pub packets: u32,
    /// The strongest it has been heard, and the last level it was heard at.
    pub best_rssi_dbfs: f32,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    pub last_us: u64,
}

impl Station {
    /// Channels this transmitter's own width sits on, its own included.
    pub fn covers(&self, channel: u16) -> bool {
        let spread = spread(self.plan);
        self.channel.abs_diff(channel) <= spread
    }
}

/// How crowded one channel is.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelLoad {
    pub plan: ChannelPlan,
    pub number: u16,
    /// Transmitters working this channel.
    pub stations: u32,
    /// Transmitters on a neighbouring channel whose width reaches this one.
    /// On 2.4 GHz that is most of the interference, and a view showing only
    /// the first number says an empty channel is a quiet one.
    pub overlapping: u32,
    pub packets: u32,
    /// The strongest anything working this channel was heard at.
    pub best_rssi_dbfs: f32,
}

/// How many channel numbers either side of its own a transmitter's width
/// reaches.
///
/// Wi-Fi numbers step 5 MHz, at 2.4 GHz and at 5 GHz alike, and a channel is
/// 20 MHz: four numbers wide, so two each side. A BLE channel is as wide as
/// the step between two of them, and an 802.15.4 one is narrower than its
/// five megahertz step, so neither reaches its neighbour.
fn spread(plan: ChannelPlan) -> u16 {
    match plan {
        ChannelPlan::Wifi => 2,
        ChannelPlan::Ble | ChannelPlan::Ieee802154 => 0,
    }
}

/// What a pane showing the channels needs, in one read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ChannelStatus {
    /// Newest heard first.
    pub stations: Vec<Station>,
    /// Every channel anything was heard working, in number order.
    pub loads: Vec<ChannelLoad>,
}

pub struct ChannelMapNode {
    stations: Vec<Station>,
    packets: u64,
}

impl Default for ChannelMapNode {
    fn default() -> Self {
        Self::new()
    }
}

impl ChannelMapNode {
    pub fn new() -> Self {
        Self { stations: Vec::new(), packets: 0 }
    }

    pub fn stations(&self) -> &[Station] {
        &self.stations
    }

    /// Packets carrying a channel statement since the node was built.
    pub fn packets(&self) -> u64 {
        self.packets
    }

    pub fn clear(&mut self) {
        self.stations.clear();
        self.packets = 0;
    }

    /// One row per channel anything is working, with what else reaches it.
    pub fn loads(&self) -> Vec<ChannelLoad> {
        let mut out: Vec<ChannelLoad> = Vec::new();
        for s in &self.stations {
            if out.iter().any(|l| l.plan == s.plan && l.number == s.channel) {
                continue;
            }
            out.push(ChannelLoad {
                plan: s.plan,
                number: s.channel,
                stations: 0,
                overlapping: 0,
                packets: 0,
                best_rssi_dbfs: f32::NEG_INFINITY,
            });
        }
        for l in &mut out {
            for s in self.stations.iter().filter(|s| s.plan == l.plan) {
                match s.channel == l.number {
                    true => {
                        l.stations += 1;
                        l.packets += s.packets;
                        l.best_rssi_dbfs = l.best_rssi_dbfs.max(s.best_rssi_dbfs);
                    }
                    false if s.covers(l.number) => l.overlapping += 1,
                    false => {}
                }
            }
        }
        out.sort_by_key(|l| (l.plan, l.number));
        out
    }

    pub fn status(&self) -> ChannelStatus {
        let mut stations = self.stations.clone();
        stations.sort_by(|a, b| b.last_us.cmp(&a.last_us));
        ChannelStatus { stations, loads: self.loads() }
    }

    /// File one decode under the transmitter that sent it.
    ///
    /// A frame naming nobody is still evidence that the channel is busy, so
    /// it counts against whatever the channel already holds rather than
    /// opening a row with no name on it.
    fn file(
        &mut self,
        p: &Packet,
        id: &str,
        name: Option<String>,
        vendor: Option<String>,
        use_: &Channel,
    ) {
        self.packets += 1;
        if let Some(s) = self.stations.iter_mut().find(|s| s.plan == use_.plan && s.id == id) {
            s.packets += 1;
            s.channel = use_.working();
            s.heard_on = use_.heard;
            s.rssi_dbfs = p.carrier.rssi_dbfs;
            s.best_rssi_dbfs = s.best_rssi_dbfs.max(p.carrier.rssi_dbfs);
            s.snr_db = p.carrier.snr_db;
            s.last_us = p.carrier.at_us;
            if name.is_some() {
                s.name = name;
            }
            if vendor.is_some() {
                s.vendor = vendor;
            }
            // A beacon says what protects the network; the data frames that
            // follow say nothing, and a row that took the latest word would
            // flip an open network to protected and back.
            if use_.secrecy != Secrecy::Unsaid {
                s.secrecy = use_.secrecy.clone();
            }
            return;
        }
        if self.stations.len() >= MAX_STATIONS
            && let Some((i, _)) = self.stations.iter().enumerate().min_by_key(|(_, s)| s.last_us)
        {
            self.stations.remove(i);
        }
        self.stations.push(Station {
            plan: use_.plan,
            id: id.to_string(),
            name,
            vendor,
            channel: use_.working(),
            heard_on: use_.heard,
            width_hz: use_.width_hz,
            secrecy: use_.secrecy.clone(),
            packets: 1,
            best_rssi_dbfs: p.carrier.rssi_dbfs,
            rssi_dbfs: p.carrier.rssi_dbfs,
            snr_db: p.carrier.snr_db,
            last_us: p.carrier.at_us,
        });
    }
}

impl Simple for ChannelMapNode {
    fn name(&self) -> &str {
        DESC.name
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("the channel map reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn params(&self) -> Vec<Param> {
        // Settable, because emptying the list is how an operator starts a
        // fresh look at a band: there is no such thing as a parameter that is
        // an action.
        vec![Param::bool("clear", false).label("Forget what was heard")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "clear" => {
                if v.as_bool().unwrap_or(false) {
                    self.clear();
                }
            }
            _ => return Err(common::Error::other(format!("no parameter {name}"))),
        }
        Ok(())
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            ("transmitters".into(), self.stations.len().to_string()),
            ("channels".into(), self.loads().len().to_string()),
        ]
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        for p in i.as_packets().unwrap_or(&[]) {
            // The channel a transmitter is working and who it is are both
            // statements a decoder made; a packet carrying neither is traffic
            // on some other plan.
            let Some(use_) = p.facts().find_map(|(_, f)| match f {
                common::packet::Fact::Channel(c) => Some(c.clone()),
                _ => None,
            }) else {
                continue;
            };
            let Some(who) = p.subject() else { continue };
            self.file(p, &who.id.to_string(), who.name.clone(), who.vendor.clone(), &use_);
        }
        Ok(())
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "channel_map",
    summary: "What is on each channel, from every decoder that names one",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(ChannelMapNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use common::packet::{Entity, Fact, Id, Proto};

    fn wifi(id: &str, ssid: Option<&str>, heard: u16, claims: Option<u16>, rssi: f32) -> Packet {
        let mut who = Entity::new("wifi", Id::Text(id.to_string()));
        who.name = ssid.map(str::to_string);
        crate::measured(2_437_000_000, 20_000_000, vec![0x80, 0x00], rssi, 20.0).decoded(
            Proto::new("wifi", "beacon").by(who).saying(Fact::Channel(
                Channel::new(ChannelPlan::Wifi, heard, 20_000_000)
                    .claiming(claims)
                    .protected_by(Secrecy::Encrypted(Some("wpa2".into()))),
            )),
        )
    }

    fn feed(node: &mut ChannelMapNode, packets: Vec<Packet>) {
        let mut s = StreamSpec::iq(20_000_000.0, Hz(2_437_000_000));
        s.kind = PortKind::Packets;
        let ins = [PortSpec { spec: s, latency: 0 }];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        node.process(&Payload::Packets(packets), &mut out, &mut ctx).unwrap();
    }

    /// One row per transmitter however many frames it sent, filed under the
    /// channel it claims rather than the one the dial was on.
    #[test]
    fn a_transmitter_is_one_row_on_the_channel_it_claims() {
        let mut n = ChannelMapNode::new();
        for _ in 0..7 {
            feed(&mut n, vec![wifi("AA:BB:CC:00:00:01", Some("home"), 6, Some(1), -52.0)]);
        }
        assert_eq!(n.stations().len(), 1);
        let s = &n.stations()[0];
        assert_eq!(s.packets, 7);
        assert_eq!(s.channel, 1, "filed under the channel it was heard on, not the one it claims");
        assert_eq!(s.heard_on, 6);
        assert_eq!(s.name.as_deref(), Some("home"));
        assert_eq!(s.secrecy, Secrecy::Encrypted(Some("wpa2".into())));
        assert_eq!(n.packets(), 7);
    }

    /// A frame that claims nothing is on the channel it was heard on, which
    /// is all a receiver can say about it.
    #[test]
    fn a_frame_that_claims_nothing_counts_where_it_was_heard() {
        let mut n = ChannelMapNode::new();
        feed(&mut n, vec![wifi("AA:BB:CC:00:00:02", None, 11, None, -70.0)]);
        assert_eq!(n.stations()[0].channel, 11);
        assert_eq!(n.stations()[0].heard_on, 11);
    }

    /// The crowding: a 20 MHz network reaches the two channels either side of
    /// its own, so a channel with nothing on it can still be busy.
    #[test]
    fn a_neighbour_two_channels_away_still_crowds_the_channel() {
        let mut n = ChannelMapNode::new();
        feed(
            &mut n,
            vec![
                wifi("AA:BB:CC:00:00:01", Some("one"), 1, Some(1), -40.0),
                wifi("AA:BB:CC:00:00:02", Some("three"), 1, Some(3), -55.0),
                wifi("AA:BB:CC:00:00:03", Some("six"), 6, Some(6), -80.0),
                wifi("AA:BB:CC:00:00:04", Some("eleven"), 11, Some(11), -60.0),
            ],
        );
        let loads = n.loads();
        assert_eq!(loads.len(), 4, "expected channels 1, 3, 6 and 11: {loads:?}");
        assert_eq!(loads.iter().map(|l| l.number).collect::<Vec<_>>(), vec![1, 3, 6, 11]);
        // Channel 1 has one network of its own and the one on 3 across it.
        assert_eq!(loads[0].stations, 1);
        assert_eq!(loads[0].overlapping, 1);
        assert_eq!(loads[0].best_rssi_dbfs, -40.0);
        // Channel 3 has its own and the one on 1.
        assert_eq!(loads[1].stations, 1);
        assert_eq!(loads[1].overlapping, 1);
        // 6 and 11 are five apart, which is more than two channels of reach
        // each: neither touches the other, and neither touches 1 or 3.
        assert_eq!(loads[2].overlapping, 0);
        assert_eq!(loads[3].overlapping, 0);
    }

    /// Two plans are two sets of numbers: channel 38 on Bluetooth is not
    /// channel 38 of anything else, and neither crowds the other.
    #[test]
    fn channels_of_different_plans_are_kept_apart() {
        let mut n = ChannelMapNode::new();
        let mut ble = wifi("AA:BB:CC:00:00:09", Some("beacon"), 38, None, -66.0);
        ble.stack = vec![
            Proto::new("ble", "adv")
                .by(Entity::new("ble", Id::Text("AA:BB:CC:00:00:09".into())))
                .saying(Fact::Channel(Channel::new(ChannelPlan::Ble, 38, 2_000_000))),
        ];
        feed(&mut n, vec![wifi("AA:BB:CC:00:00:01", Some("home"), 6, Some(6), -50.0), ble]);
        let loads = n.loads();
        assert_eq!(loads.len(), 2);
        assert_eq!((loads[0].plan, loads[0].number), (ChannelPlan::Wifi, 6));
        assert_eq!((loads[1].plan, loads[1].number), (ChannelPlan::Ble, 38));
        // A BLE channel is as wide as the step between two of them, so it
        // crowds nothing beside it.
        assert_eq!(loads[1].stations, 1);
        assert_eq!(loads[1].overlapping, 0);
    }

    /// A decode with no channel statement is not this view's business,
    /// however much of it arrives.
    #[test]
    fn packets_without_a_channel_are_not_counted() {
        let mut n = ChannelMapNode::new();
        let mut p = wifi("AA:BB:CC:00:00:01", Some("home"), 6, Some(6), -50.0);
        p.stack = vec![Proto::new("pocsag", "alpha")];
        for _ in 0..200 {
            feed(&mut n, vec![p.clone()]);
        }
        assert_eq!(n.stations().len(), 0);
        assert_eq!(n.loads().len(), 0);
        assert_eq!(n.packets(), 0);
    }

    /// Clearing forgets everything, which is how a fresh look at a band
    /// starts.
    #[test]
    fn clearing_forgets_every_transmitter() {
        let mut n = ChannelMapNode::new();
        feed(&mut n, vec![wifi("AA:BB:CC:00:00:01", Some("home"), 6, Some(6), -50.0)]);
        assert_eq!(n.stations().len(), 1);
        n.set_param("clear", ParamValue::Bool(true)).unwrap();
        assert_eq!(n.stations().len(), 0);
        assert_eq!(n.status(), ChannelStatus::default());
    }

    /// The strongest level is kept, not the last one: a network heard once
    /// from the doorstep and since from the far end of the garden is still
    /// the loud one.
    #[test]
    fn the_strongest_level_is_kept_beside_the_latest() {
        let mut n = ChannelMapNode::new();
        feed(&mut n, vec![wifi("AA:BB:CC:00:00:01", Some("home"), 6, Some(6), -38.0)]);
        feed(&mut n, vec![wifi("AA:BB:CC:00:00:01", Some("home"), 6, Some(6), -77.0)]);
        assert_eq!(n.stations()[0].best_rssi_dbfs, -38.0);
        assert_eq!(n.stations()[0].rssi_dbfs, -77.0);
        assert_eq!(n.stations()[0].packets, 2);
    }
}
