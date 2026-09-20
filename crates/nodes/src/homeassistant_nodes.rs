//! Publishing what was heard to Home Assistant, as a node on the packet bus.
//!
//! A receiver that reads a hundred sensors in the neighbourhood and keeps the
//! readings to itself is a spectrum analyser. This is the way out to the
//! house: every transmitter the decoders name becomes a device in Home
//! Assistant, and every number they recover becomes an entity under it,
//! through MQTT discovery.
//!
//! # Discovery, so nothing is configured twice
//!
//! Home Assistant builds entities from retained configuration messages under
//! its discovery prefix, so a device heard once is in the house for good
//! without anybody editing YAML. Each device gets one state topic carrying
//! every field as JSON, and each field's configuration points a template at
//! it: one publication per reception rather than one per entity, which for a
//! weather station with nine fields is nine times less traffic.
//!
//! # One connection for the process
//!
//! The graph is rebuilt on every retune, so a node lives for seconds while a
//! broker connection wants to live for days. The connection is therefore a
//! process-wide thread that the node publishes through, not something the
//! node owns. What the node owns is which devices it has announced, so a
//! rebuild costs a re-announcement and nothing else.
//!
//! # What is not published
//!
//! A burst nothing identified. The bus carries every unclaimed burst in the
//! band, and a device per unknown OOK pulse train would fill a house with
//! entities nobody can name. The rule is the survey's rule: a transmitter
//! with an identity, and nothing else.
//!
//! # The receiver itself is a device too
//!
//! Everything published hangs off a bridge device called WaveShark, which is
//! announced as soon as the broker accepts a connection. Without it the
//! `via_device` on every other device pointed at nothing, so Home Assistant
//! dropped the link and a house full of discovered sensors had no way to say
//! where they came from.
//!
//! Beside it is the call bus: what the radio hears people saying, as an event
//! entity to trigger on and a lamp that is lit while somebody is talking, and
//! the message bus, which is the same for what people write. Neither is a
//! transmitter, so neither goes through the device-per-identity path above:
//! a talkgroup is not a thing in a house, and one entity per talkgroup on a
//! busy trunked network is a house nobody can read.

use common::Result;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// How long the broker holds a reading before Home Assistant shows the entity
/// as unavailable. Long enough for a sensor that reports twice an hour, short
/// enough that a device carried out of range stops reading as present.
const EXPIRE_AFTER_S: u64 = 3 * 3600;

/// The least time between two publications about one device.
///
/// A BLE beacon advertises ten times a second and a tyre sensor repeats its
/// frame three times a transmission; a house does not want either at that
/// rate. Overridable per node, since a bench wants to see every one.
const MIN_INTERVAL_S: f64 = 10.0;

/// Devices one node will announce. A guard rather than a working limit: a
/// city centre holds thousands of BLE addresses, and filling Home Assistant
/// with them is a mistake that takes an afternoon to undo.
const MAX_DEVICES: usize = 250;

/// Below this peak a block off the tap is silence rather than somebody
/// talking. The same floor the tap itself uses to decide who is on air.
const VOICE_FLOOR: f32 = 0.004;

/// What the bridge device is called, and the identifier every other device
/// points at with `via_device`.
const HUB: &str = "waveshark";
const HUB_NAME: &str = "WaveShark";

/// The device the call bus publishes as, and the one the messages do.
const CALL_BUS: &str = "waveshark_call_bus";
const MESSAGE_BUS: &str = "waveshark_messages";

/// How long after the last voice frame a call is still on the air.
///
/// A trunked talkgroup holds its channel for a few seconds between
/// transmissions, and an automation that fired twice for one conversation is
/// worse than one that fires a little late. The call list uses the same six
/// seconds for the same reason.
const CALL_HANG_S: f64 = 6.0;

/// A Home Assistant state is a short string, and a pager message is not
/// always short. The state carries the beginning and the attribute carries
/// what was written.
const STATE_MAX: usize = 255;

/// Entities under one device. A decoder that emits forty timing fields is
/// describing a burst, not a thing in a house.
const MAX_FIELDS: usize = 16;

/// How long the publisher waits before trying a refused broker again, and the
/// ceiling that backoff climbs to. A wrong password fails every time and must
/// not become a connection attempt a second.
const RETRY: Duration = Duration::from_secs(5);
const RETRY_MAX: Duration = Duration::from_secs(2 * 60);

/// Messages held for the broker before the oldest are dropped. Publishing
/// happens from the radio thread, which may never block on a network: a full
/// queue is a dropped reading, and the count of them is on screen.
const QUEUE: usize = 256;

/// Where to publish, as the operator set it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Broker {
    pub host: String,
    pub port: u16,
    pub username: String,
    pub password: String,
    /// Home Assistant's discovery prefix, `homeassistant` unless somebody
    /// changed it in their configuration.
    pub prefix: String,
    /// What this receiver's own topics live under.
    pub topic: String,
}

impl Broker {
    /// The defaults a fresh installation of Home Assistant and Mosquitto
    /// agree on, so an operator types a hostname and nothing else.
    pub fn new(host: &str) -> Self {
        Self {
            host: host.to_string(),
            port: 1883,
            username: String::new(),
            password: String::new(),
            prefix: "homeassistant".into(),
            topic: "waveshark".into(),
        }
    }

    /// Whether there is enough here to connect at all.
    pub fn is_complete(&self) -> bool {
        !self.host.trim().is_empty() && self.port > 0
    }

    fn prefix(&self) -> &str {
        if self.prefix.trim().is_empty() { "homeassistant" } else { self.prefix.trim() }
    }

    fn topic(&self) -> &str {
        if self.topic.trim().is_empty() { "waveshark" } else { self.topic.trim() }
    }

    /// What the broker is told to publish if this receiver disappears, and
    /// what every entity's availability points at.
    fn availability(&self) -> String {
        format!("{}/status", self.topic())
    }
}

/// Where to publish and what to publish there, as the operator set it.
///
/// The two travel together because they are one decision: pointing the feed
/// at a house is also deciding how much of the street goes into it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Publish {
    pub broker: Broker,
    /// Identity spaces worth an entity, comma separated, or empty for all of
    /// them. `ism,wmbus` is a house's own sensors and meters.
    pub spaces: String,
    /// Whether what people say and write goes to the house as well as what
    /// the sensors report.
    pub buses: bool,
}

/// What the feed is doing, for the interface to draw.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HomeAssistantStatus {
    /// Whether a broker has been set at all.
    pub configured: bool,
    pub connected: bool,
    pub host: String,
    /// Devices announced since the connection came up.
    pub devices: u64,
    pub published: u64,
    /// Readings thrown away because the queue to the broker was full, which
    /// is a broker that cannot keep up rather than a decoder that failed.
    pub dropped: u64,
    /// Why the connection is not up, when it is not.
    pub error: Option<String>,
}

/// The thread that holds the connection, and everything it reports.
pub struct Publisher {
    broker: Mutex<Option<Broker>>,
    client: Mutex<Option<rumqttc::Client>>,
    /// Bumped on every connection, so a node re-announces its devices to a
    /// broker that has restarted and lost the retained configurations.
    generation: AtomicU64,
    connected: AtomicBool,
    published: AtomicU64,
    dropped: AtomicU64,
    error: Mutex<Option<String>>,
    started: AtomicBool,
    /// Woken when the broker changes, so the thread does not sleep out a
    /// retry before trying the address it was just given.
    wake: std::sync::Condvar,
    /// Everything that was offered to the broker, for a test to read. What is
    /// worth asserting here is what would be said, and saying it needs a
    /// network.
    #[cfg(test)]
    said: Mutex<Vec<(String, String)>>,
}

impl Publisher {
    /// A publisher with no thread behind it, which is what a test wants: the
    /// messages are testable, the network is not.
    pub fn inert() -> Arc<Self> {
        Arc::new(Publisher {
            broker: Mutex::new(None),
            client: Mutex::new(None),
            generation: AtomicU64::new(0),
            connected: AtomicBool::new(false),
            published: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            error: Mutex::new(None),
            started: AtomicBool::new(false),
            wake: std::sync::Condvar::new(),
            #[cfg(test)]
            said: Mutex::new(Vec::new()),
        })
    }

    /// A publisher with its connection thread running.
    ///
    /// One per receiver, owned by it and lent to every node in its graph,
    /// so a rebuild keeps the connection and two receivers in one process
    /// (two tests, say) do not take turns setting each other's broker.
    /// It was a process-wide singleton, and a test building a receiver with
    /// no broker cleared the broker of the one that had.
    pub fn running() -> Arc<Self> {
        let p = Self::inert();
        p.start();
        p
    }

    fn start(self: &Arc<Self>) {
        if self.started.swap(true, Ordering::SeqCst) {
            return;
        }
        let p = self.clone();
        let _ = std::thread::Builder::new().name("ha-mqtt".into()).spawn(move || p.run());
    }

    /// Where to publish, or `None` to stop. Changing it drops the connection,
    /// which the thread then makes again to the new address.
    pub fn set_broker(&self, broker: Option<Broker>) {
        let want = broker.filter(Broker::is_complete);
        let mut held = match self.broker.lock() {
            Ok(h) => h,
            Err(_) => return,
        };
        if *held == want {
            return;
        }
        *held = want;
        self.disconnect();
        self.wake.notify_all();
    }

    pub fn broker(&self) -> Option<Broker> {
        self.broker.lock().ok().and_then(|b| b.clone())
    }

    fn disconnect(&self) {
        self.connected.store(false, Ordering::Relaxed);
        if let Ok(mut c) = self.client.lock()
            && let Some(client) = c.take()
        {
            let _ = client.try_disconnect();
        }
    }

    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Relaxed)
    }

    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }

    pub fn status(&self) -> HomeAssistantStatus {
        let broker = self.broker();
        HomeAssistantStatus {
            configured: broker.is_some(),
            connected: self.is_connected(),
            host: broker.as_ref().map(|b| format!("{}:{}", b.host, b.port)).unwrap_or_default(),
            devices: 0,
            published: self.published.load(Ordering::Relaxed),
            dropped: self.dropped.load(Ordering::Relaxed),
            error: self.error.lock().ok().and_then(|e| e.clone()),
        }
    }

    /// Hand one message to the broker without waiting for it.
    ///
    /// `try_publish` rather than `publish`: this is called from the radio
    /// thread, which has to keep draining USB, and a broker that has stopped
    /// reading must cost a dropped reading rather than a dropped block.
    pub fn send(&self, topic: &str, payload: String, retain: bool) {
        #[cfg(test)]
        if let Ok(mut said) = self.said.lock() {
            said.push((topic.to_string(), payload.clone()));
        }
        let client = match self.client.lock() {
            Ok(c) => c.clone(),
            Err(_) => None,
        };
        let Some(client) = client else {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let sent = client.try_publish(topic, rumqttc::QoS::AtMostOnce, retain, payload);
        match sent {
            Ok(()) => self.published.fetch_add(1, Ordering::Relaxed),
            Err(_) => self.dropped.fetch_add(1, Ordering::Relaxed),
        };
    }

    fn run(&self) {
        let mut wait = RETRY;
        loop {
            let Some(broker) = self.broker() else {
                self.pause(RETRY);
                continue;
            };
            match self.connect(&broker) {
                Ok(()) => wait = RETRY,
                Err(e) => {
                    if let Ok(mut held) = self.error.lock() {
                        *held = Some(e);
                    }
                    self.pause(wait);
                    wait = (wait * 2).min(RETRY_MAX);
                }
            }
            self.connected.store(false, Ordering::Relaxed);
        }
    }

    /// Wait out a retry, or until the broker is changed under us.
    fn pause(&self, for_: Duration) {
        if let Ok(guard) = self.broker.lock() {
            let _ = self.wake.wait_timeout(guard, for_);
        }
    }

    /// Hold one connection until it fails, which is what the loop above
    /// treats as a reason to wait and try again.
    fn connect(&self, broker: &Broker) -> std::result::Result<(), String> {
        let id = format!("waveshark-{}", std::process::id());
        let mut opts = rumqttc::MqttOptions::new(id, broker.host.trim(), broker.port);
        opts.set_keep_alive(Duration::from_secs(30));
        opts.set_max_packet_size(64 * 1024, 64 * 1024);
        if !broker.username.is_empty() {
            opts.set_credentials(broker.username.clone(), broker.password.clone());
        }
        // What the broker says on this receiver's behalf if it stops saying
        // anything: every entity's availability points here, so a receiver
        // that was switched off reads as unavailable rather than as a house
        // full of sensors stuck at their last value.
        opts.set_last_will(rumqttc::LastWill::new(
            broker.availability(),
            "offline",
            rumqttc::QoS::AtLeastOnce,
            true,
        ));
        let (client, mut conn) = rumqttc::Client::new(opts, QUEUE);
        for event in conn.iter() {
            match event {
                Ok(rumqttc::Event::Incoming(rumqttc::Packet::ConnAck(ack))) => {
                    if ack.code != rumqttc::ConnectReturnCode::Success {
                        return Err(format!("the broker refused the connection: {:?}", ack.code));
                    }
                    // The client is only published to the nodes once the
                    // broker has accepted: a queue filling up against a
                    // connection that was refused is readings thrown away
                    // with nowhere to go.
                    if let Ok(mut held) = self.client.lock() {
                        *held = Some(client.clone());
                    }
                    let _ = client.try_publish(
                        broker.availability(),
                        rumqttc::QoS::AtLeastOnce,
                        true,
                        "online",
                    );
                    self.generation.fetch_add(1, Ordering::Relaxed);
                    self.connected.store(true, Ordering::Relaxed);
                    if let Ok(mut e) = self.error.lock() {
                        *e = None;
                    }
                }
                Ok(_) => {}
                Err(e) => {
                    self.disconnect();
                    return Err(e.to_string());
                }
            }
            // The operator changed the address, so this connection is to the
            // wrong place whatever it is doing.
            if self.broker().as_ref() != Some(broker) {
                self.disconnect();
                return Ok(());
            }
        }
        self.disconnect();
        Ok(())
    }
}

/// What has been said about one device, so it is not said again.
struct Known {
    /// Fields already given a configuration message.
    announced: HashSet<String>,
    /// The name and vendor those messages carried. A device is usually first
    /// heard from a frame that says neither, and the frame that names it
    /// comes later; the configurations go out again when it does, since the
    /// device block is in them and nowhere else.
    named: (Option<String>, Option<String>),
    /// The connection those messages were sent over. A broker that restarted
    /// has lost them, and its generation says so.
    generation: u64,
    last: Instant,
}

/// A call in progress, as the house is told about it.
#[derive(Clone, Debug, PartialEq)]
struct OnAir {
    system: String,
    channel_hz: f64,
    to: String,
    from: Option<String>,
    /// The coded squelch an analogue channel's users are set to, where the
    /// audio said: "141.3" or "D023".
    code: Option<String>,
    encrypted: bool,
    codec: Option<&'static str>,
    started: Instant,
    last: Instant,
}

/// The feed to Home Assistant, on the packet bus and on the tap.
pub struct HomeAssistantNode {
    publisher: Arc<Publisher>,
    known: HashMap<(String, String), Known>,
    /// Calls being talked on, by system, channel and group. Cleared by the
    /// hang, which is what publishes `call_ended`.
    on_air: Vec<OnAir>,
    /// The connection the bus devices were announced over, so a broker that
    /// restarted is introduced to them again and a running one is not.
    announced_buses: Option<u64>,
    devices: u64,
    /// The least time between two publications about one device.
    min_interval: Duration,
    max_devices: usize,
    /// Whether what people say and write is published at all.
    ///
    /// A house may want its own meters and not the radio traffic: an
    /// operator listening to somebody else's network has a reason to keep it
    /// off the dashboard, and a broker is not a private place.
    buses: bool,
    /// Identity spaces worth a permanent entity, or empty for all of them.
    ///
    /// The reason this exists is the phone in the street. A BLE address is
    /// resolvable-private and rotates every quarter of an hour, so a receiver
    /// left running publishes a new device four times an hour for every
    /// handset that walks past, and Home Assistant keeps every one of them
    /// for good. A house that wants its own sensors sets this to `ism,wmbus`
    /// and gets the meters without the pedestrians.
    spaces: Vec<String>,
}

impl Default for HomeAssistantNode {
    /// A node with nowhere to publish until a receiver lends it a publisher
    /// (`set_publisher`). Built from the registry this way, since the
    /// registry cannot know which receiver the stage is for.
    fn default() -> Self {
        Self::with(Publisher::inert())
    }
}

impl HomeAssistantNode {
    pub fn with(publisher: Arc<Publisher>) -> Self {
        Self {
            publisher,
            known: HashMap::new(),
            on_air: Vec::new(),
            buses: true,
            announced_buses: None,
            devices: 0,
            min_interval: Duration::from_secs_f64(MIN_INTERVAL_S),
            max_devices: MAX_DEVICES,
            spaces: Vec::new(),
        }
    }

    /// Whether a transmitter's identity space is one of the wanted ones.
    ///
    /// A prefix rather than an exact word, because a space names the model as
    /// well as the system: every rtl_433 style sensor is `ism:<model>`, and
    /// `ism` is how somebody asks for all of them.
    fn wanted(&self, space: &str) -> bool {
        self.spaces.is_empty() || self.spaces.iter().any(|w| space.starts_with(w.as_str()))
    }

    pub fn set_broker(&mut self, broker: Option<Broker>) {
        self.publisher.set_broker(broker);
    }

    /// Publish through this publisher from now on. What was announced over
    /// the old one is forgotten, since the new one's broker has not heard it.
    pub fn set_publisher(&mut self, p: Arc<Publisher>) {
        if !Arc::ptr_eq(&self.publisher, &p) {
            self.publisher = p;
            self.known.clear();
            self.announced_buses = None;
        }
    }

    /// Which identity spaces to publish, as a comma-separated list, or empty
    /// for every one of them.
    /// Whether calls and messages are published at all.
    pub fn set_buses(&mut self, on: bool) {
        self.buses = on;
    }

    pub fn set_spaces(&mut self, spaces: &str) {
        self.spaces =
            spaces.split(',').map(|s| s.trim().to_lowercase()).filter(|s| !s.is_empty()).collect();
        if self.spaces.iter().any(|s| s == "all") {
            self.spaces.clear();
        }
    }

    pub fn is_on(&self) -> bool {
        self.publisher.broker().is_some()
    }

    pub fn status(&self) -> HomeAssistantStatus {
        HomeAssistantStatus { devices: self.devices, ..self.publisher.status() }
    }

    /// Announce the bridge, the call bus and the message bus, once per
    /// connection.
    ///
    /// The bridge first and with an entity of its own, because a device with
    /// no entities is a device Home Assistant does not keep, and every other
    /// device's `via_device` points at this one.
    fn announce_buses(&mut self) {
        if !self.buses {
            return;
        }
        let Some(broker) = self.publisher.broker() else { return };
        let generation = self.publisher.generation();
        if self.announced_buses == Some(generation) {
            return;
        }
        self.announced_buses = Some(generation);
        let hub = serde_json::json!({
            "identifiers": [HUB],
            "name": HUB_NAME,
            "manufacturer": HUB_NAME,
            "model": "Wideband receiver",
            "sw_version": env!("CARGO_PKG_VERSION"),
        });
        let availability = broker.availability();
        // The bridge's own entity, which is also the honest answer to "is the
        // receiver still there": it is the last will the broker publishes
        // when this process disappears.
        self.publisher.send(
            &format!("{}/binary_sensor/{HUB}/receiving/config", broker.prefix()),
            serde_json::json!({
                "name": "Receiving",
                "unique_id": format!("{HUB}_receiving"),
                "object_id": format!("{HUB}_receiving"),
                "state_topic": availability,
                "payload_on": "online",
                "payload_off": "offline",
                "device_class": "connectivity",
                "entity_category": "diagnostic",
                "device": hub,
            })
            .to_string(),
            true,
        );

        let calls = format!("{}/calls", broker.topic());
        let bus_device = |id: &str, name: &str| {
            serde_json::json!({
                "identifiers": [id],
                "name": name,
                "manufacturer": HUB_NAME,
                "model": "Air",
                "via_device": HUB,
            })
        };
        // The event entity is what an automation triggers on: it fires and
        // leaves no state behind, which is right for something that happened
        // rather than something that is.
        self.publisher.send(
            &format!("{}/event/{CALL_BUS}/call/config", broker.prefix()),
            serde_json::json!({
                "name": "Call",
                "unique_id": format!("{CALL_BUS}_call"),
                "object_id": format!("{CALL_BUS}_call"),
                "state_topic": format!("{calls}/event"),
                "event_types": ["call_started", "call_ended"],
                "availability_topic": availability,
                "device": bus_device(CALL_BUS, "WaveShark call bus"),
            })
            .to_string(),
            true,
        );
        self.publisher.send(
            &format!("{}/binary_sensor/{CALL_BUS}/on_air/config", broker.prefix()),
            serde_json::json!({
                "name": "On air",
                "unique_id": format!("{CALL_BUS}_on_air"),
                "object_id": format!("{CALL_BUS}_on_air"),
                "state_topic": format!("{calls}/state"),
                "value_template": "{{ value_json.on_air }}",
                "payload_on": "ON",
                "payload_off": "OFF",
                "device_class": "sound",
                "availability_topic": availability,
                "device": bus_device(CALL_BUS, "WaveShark call bus"),
            })
            .to_string(),
            true,
        );
        // The last of each, because an event entity's attributes are awkward
        // to put on a dashboard and "who was that" is the question somebody
        // asks of a scanner.
        for (field, name, unit) in [
            ("caller", "Last caller", None),
            ("talkgroup", "Last talkgroup", None),
            ("system", "Last system", None),
            ("channel_mhz", "Last channel", Some("MHz")),
        ] {
            let mut config = serde_json::json!({
                "name": name,
                "unique_id": format!("{CALL_BUS}_{field}"),
                "object_id": format!("{CALL_BUS}_{field}"),
                "state_topic": format!("{calls}/state"),
                "value_template": format!("{{{{ value_json.{field} }}}}"),
                "availability_topic": availability,
                "device": bus_device(CALL_BUS, "WaveShark call bus"),
            });
            if let Some(u) = unit {
                config["unit_of_measurement"] = serde_json::json!(u);
                config["device_class"] = serde_json::json!("frequency");
                config["state_class"] = serde_json::json!("measurement");
            }
            self.publisher.send(
                &format!("{}/sensor/{CALL_BUS}/{field}/config", broker.prefix()),
                config.to_string(),
                true,
            );
        }

        let messages = format!("{}/messages", broker.topic());
        self.publisher.send(
            &format!("{}/event/{MESSAGE_BUS}/message/config", broker.prefix()),
            serde_json::json!({
                "name": "Message",
                "unique_id": format!("{MESSAGE_BUS}_message"),
                "object_id": format!("{MESSAGE_BUS}_message"),
                "state_topic": format!("{messages}/event"),
                "event_types": ["message"],
                "availability_topic": availability,
                "device": bus_device(MESSAGE_BUS, "WaveShark messages"),
            })
            .to_string(),
            true,
        );
        // The words themselves are an attribute rather than the state: a
        // state is 255 characters and a page can be longer, and a truncated
        // message is a message misread.
        self.publisher.send(
            &format!("{}/sensor/{MESSAGE_BUS}/last/config", broker.prefix()),
            serde_json::json!({
                "name": "Last message",
                "unique_id": format!("{MESSAGE_BUS}_last"),
                "object_id": format!("{MESSAGE_BUS}_last"),
                "state_topic": format!("{messages}/state"),
                "value_template": "{{ value_json.text }}",
                "json_attributes_topic": format!("{messages}/state"),
                "availability_topic": availability,
                "device": bus_device(MESSAGE_BUS, "WaveShark messages"),
            })
            .to_string(),
            true,
        );
    }

    /// What the call bus says about itself: whether anybody is talking, and
    /// who was heard last.
    ///
    /// Retained, unlike a reading: this is a state rather than a moment, and
    /// Home Assistant restarted should know whether the channel is busy
    /// without waiting for somebody to key up.
    fn publish_call_state(&self, call: Option<&OnAir>) {
        let Some(broker) = self.publisher.broker() else { return };
        let state = match call {
            Some(c) => serde_json::json!({
                "on_air": "ON",
                "caller": c.from.clone().unwrap_or_default(),
                "talkgroup": c.to,
                "system": c.system,
                "channel_mhz": (c.channel_hz / 1e6 * 10_000.0).round() / 10_000.0,
                "code": c.code.clone().unwrap_or_default(),
            }),
            // The last caller stays: what a scanner is asked when it is quiet
            // is who that was, not who nobody is.
            None => serde_json::json!({ "on_air": "OFF" }),
        };
        self.publisher.send(&format!("{}/calls/state", broker.topic()), state.to_string(), true);
    }

    fn publish_call_event(&self, kind: &str, c: &OnAir, now: Instant) {
        let Some(broker) = self.publisher.broker() else { return };
        let event = serde_json::json!({
            "event_type": kind,
            "system": c.system,
            "talkgroup": c.to,
            "caller": c.from.clone().unwrap_or_default(),
            "channel_mhz": (c.channel_hz / 1e6 * 10_000.0).round() / 10_000.0,
            "encrypted": c.encrypted,
            "codec": c.codec.unwrap_or_default(),
            "code": c.code.clone().unwrap_or_default(),
            "seconds": (now.saturating_duration_since(c.started).as_secs_f64() * 10.0).round() / 10.0,
        });
        self.publisher.send(&format!("{}/calls/event", broker.topic()), event.to_string(), false);
    }

    /// A block of speech off the tap, which is where every conversation the
    /// receiver hears passes, analogue or decoded.
    ///
    /// Audio says who is talking to whom and on what, and nothing else: no
    /// codec, and no word on whether it was enciphered. A decode of the same
    /// call fills those in.
    ///
    /// Silence is not a call. A squelched channel and a vocoder between overs
    /// both deliver blocks of nothing, and publishing them would light the
    /// on-air lamp for the length of the session.
    fn hear_voice(&mut self, v: &common::Voice, now: Instant) {
        if !self.buses {
            return;
        }
        let Some(to) = v.to.as_deref().map(str::trim).filter(|t| !t.is_empty()) else { return };
        // A system that counts the channel for itself is talking whether or
        // not the speech could be decoded: nothing here reads IMBE, and a
        // house watching a P25 network would otherwise never see a call.
        let stated = v.over.as_ref().is_some_and(|o| o.seconds > 0.0);
        if !stated && v.pcm.iter().all(|s| s.abs() <= VOICE_FLOOR) {
            return;
        }
        let system = v.system.to_string();
        let channel_hz = v.channel_hz;
        let found = self.on_air.iter_mut().find(|c| {
            c.system == system && c.to == to && (c.channel_hz - channel_hz).abs() < 500.0
        });
        if let Some(c) = found {
            c.last = now;
            let mut news = false;
            if c.from.is_none() && v.from.is_some() {
                c.from = v.from.clone();
                news = true;
            }
            // The group takes half a second of audio to read, so it lands
            // after the call was published, and a code changed on the radio
            // replaces it: the house is told once, not per block.
            if v.code.is_some() && c.code != v.code {
                c.code = v.code.clone();
                news = true;
            }
            // What the system said about the call: audio names nobody and
            // says nothing about a cipher, so the over fills it in and the
            // house is told once rather than on every block.
            if let Some(o) = v.over.as_ref() {
                if c.codec.is_none() && o.codec.is_some() {
                    c.codec = o.codec;
                    news = true;
                }
                if o.encrypted() && !c.encrypted {
                    c.encrypted = true;
                    news = true;
                }
            }
            if news {
                let call = c.clone();
                self.publish_call_state(Some(&call));
            }
            return;
        }
        let call = OnAir {
            system,
            channel_hz,
            to: to.to_string(),
            from: v.from.clone(),
            code: v.code.clone(),
            encrypted: v.over.as_ref().is_some_and(|o| o.encrypted()),
            codec: v.over.as_ref().and_then(|o| o.codec),
            started: now,
            last: now,
        };
        self.publish_call_event("call_started", &call, now);
        self.publish_call_state(Some(&call));
        self.on_air.push(call);
    }

    /// Close the calls nothing has been heard on for the hang.
    fn age_calls(&mut self, now: Instant) {
        let hang = Duration::from_secs_f64(CALL_HANG_S);
        let ended: Vec<OnAir> = self
            .on_air
            .iter()
            .filter(|c| now.saturating_duration_since(c.last) >= hang)
            .cloned()
            .collect();
        if ended.is_empty() {
            return;
        }
        self.on_air.retain(|c| now.saturating_duration_since(c.last) < hang);
        for c in &ended {
            self.publish_call_event("call_ended", c, c.last);
        }
        self.publish_call_state(self.on_air.last());
    }

    /// A message somebody wrote, as an event and as the last one.
    fn hear_message(&mut self, p: &common::packet::Packet) {
        if !self.buses {
            return;
        }
        let Some(broker) = self.publisher.broker() else { return };
        let Some((layer, text)) = p.facts().find_map(|(l, f)| match f {
            common::packet::Fact::Message(w) => Some((l, w.text.clone())),
            _ => None,
        }) else {
            return;
        };
        if text.trim().is_empty() {
            return;
        }
        let party = |e: &Option<common::packet::Party>| {
            e.as_ref().map(|q| q.label().to_string()).unwrap_or_default()
        };
        let body = serde_json::json!({
            "system": layer.id,
            "from": party(&layer.link.from),
            "to": party(&layer.link.to),
            "text": text.chars().take(STATE_MAX).collect::<String>(),
            "full_text": text,
            "channel_mhz": (p.carrier.center_hz as f64 / 1e6 * 10_000.0).round() / 10_000.0,
        });
        let mut event = body.clone();
        event["event_type"] = serde_json::json!("message");
        let topic = broker.topic();
        self.publisher.send(&format!("{topic}/messages/event"), event.to_string(), false);
        self.publisher.send(&format!("{topic}/messages/state"), body.to_string(), true);
    }

    /// One decode, as a device in a house.
    fn publish(&mut self, p: &common::packet::Packet, now: Instant) {
        let Some((space, ident)) = crate::survey_nodes::identity(p) else { return };
        if !self.wanted(&space) {
            return;
        }
        let key = (space.clone(), ident.clone());
        let generation = self.publisher.generation();
        if let Some(known) = self.known.get(&key) {
            if known.generation == generation && now.duration_since(known.last) < self.min_interval
            {
                return;
            }
        } else if self.known.len() >= self.max_devices {
            return;
        }

        let Some(broker) = self.publisher.broker() else { return };
        let node_id = format!("waveshark_{}_{}", slug(&space), slug(&ident));
        let state_topic = format!("{}/{}/{}/state", broker.topic(), slug(&space), slug(&ident));

        let readings = readings(p);
        let fresh = !self.known.contains_key(&key);
        let entry = self.known.entry(key).or_insert_with(|| Known {
            announced: HashSet::new(),
            named: (None, None),
            generation,
            last: now,
        });
        // A broker that restarted has forgotten every retained configuration,
        // so a new connection is a new introduction.
        if entry.generation != generation {
            entry.announced.clear();
            entry.generation = generation;
        }
        // A name learned since the last announcement is worth announcing
        // again: what was said was "BLE e8:31:cd", and the house should
        // read "Kitchen scale". A name that goes away is not unlearned.
        let name = crate::survey_nodes::name_of(p).or_else(|| entry.named.0.clone());
        let vendor = crate::survey_nodes::vendor_of(p).or_else(|| entry.named.1.clone());
        if (name.as_ref(), vendor.as_ref()) != (entry.named.0.as_ref(), entry.named.1.as_ref()) {
            entry.announced.clear();
            entry.named = (name.clone(), vendor.clone());
        }
        for (field, value, unit) in &readings {
            if entry.announced.len() >= MAX_FIELDS || entry.announced.contains(field) {
                continue;
            }
            if !matches!(value, common::Value::Float(_) | common::Value::Int(_)) {
                continue;
            }
            entry.announced.insert(field.clone());
            let config = discovery(
                &broker,
                &node_id,
                &state_topic,
                field,
                unit.as_deref(),
                &space,
                &ident,
                name.as_deref(),
                vendor.as_deref(),
            );
            self.publisher.send(
                &format!("{}/sensor/{node_id}/{}/config", broker.prefix(), slug(field)),
                config,
                true,
            );
        }
        if fresh {
            self.devices += 1;
        }
        entry.last = now;
        // Not retained: a reading is a moment, the entity expires it, and a
        // retained state is one more message the broker keeps and replays
        // for every device that was ever heard.
        self.publisher.send(&state_topic, state(&readings), false);
    }
}

impl Node for HomeAssistantNode {
    fn name(&self) -> &str {
        "homeassistant"
    }

    fn is_sink(&self) -> bool {
        true
    }

    /// The packet bus, and the tap every voice passes.
    ///
    /// Two inputs because a call and a device are heard by different halves
    /// of the receiver: a meter is a packet, and a conversation is audio. On
    /// the packet bus alone the call bus was silent on a receiver with no
    /// decoder running, which is every receiver listening to an analogue
    /// channel.
    fn num_inputs(&self) -> usize {
        2
    }

    /// Either input on its own is a receiver worth publishing: a span with no
    /// decoder has no packet bus, and a span with no voice channel has no
    /// tap.
    fn optional_inputs(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        for (k, i) in inputs.iter().enumerate() {
            let ok = match k {
                0 => i.spec.kind == PortKind::Packets,
                _ => i.spec.kind == PortKind::Voice,
            };
            if !ok && !i.spec.is_silence() {
                return Err(common::Error::other(format!(
                    "homeassistant reads the packet bus and the tap, and input {k} carries {:?}",
                    i.spec.kind
                )));
            }
        }
        // A sink, so what it declares is silence: the graph gives every node
        // a slot whether or not anything reads it.
        Ok(vec![StreamSpec::silence()])
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        vec![
            pipeline::param::Param::float(
                "min_interval_s",
                self.min_interval.as_secs_f64(),
                0.0..=600.0,
            )
            .unit("s"),
            pipeline::param::Param::int("max_devices", self.max_devices as i64, 1..=5_000),
            pipeline::param::Param::text("spaces", self.spaces.join(",")),
            pipeline::param::Param::bool("buses", self.buses).label("Publish calls and messages"),
        ]
    }

    fn set_param(&mut self, name: &str, v: pipeline::param::ParamValue) -> Result<()> {
        match name {
            "min_interval_s" => {
                if let Some(s) = v.as_f64() {
                    self.min_interval = Duration::from_secs_f64(s.max(0.0));
                }
                Ok(())
            }
            "max_devices" => {
                if let Some(n) = v.as_i64() {
                    self.max_devices = n.max(1) as usize;
                }
                Ok(())
            }
            "spaces" => {
                self.set_spaces(v.as_str().unwrap_or(""));
                Ok(())
            }
            "buses" => {
                self.buses = v.as_bool().unwrap_or(true);
                Ok(())
            }
            _ => Err(common::Error::other(format!("no parameter {name}"))),
        }
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        _o: &mut [Payload],
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        if !self.is_on() {
            return Ok(());
        }
        let now = Instant::now();
        self.announce_buses();
        for p in inputs.first().and_then(|i| i.as_packets()).unwrap_or(&[]) {
            self.hear_message(p);
            self.publish(p, now);
        }
        // Every conversation the receiver hears, decoded or analogue, arrives
        // here as audio. A decoded call is on both inputs and is one call:
        // whichever reaches it first opens the row, and the decode fills in
        // what audio cannot say.
        for v in inputs.get(1).and_then(|i| i.as_voice()).unwrap_or(&[]) {
            self.hear_voice(v, now);
        }
        // Called every block whether or not anything arrived: a call ends
        // when nothing more is heard on it, and silence is not a packet.
        self.age_calls(now);
        Ok(())
    }
}

/// What is worth publishing about one reception: what the decoder measured,
/// and how strongly it was heard.
///
/// The level is here rather than left out because it is the one reading every
/// device has, and it is what says a sensor is going out of range before it
/// stops reporting altogether.
fn readings(p: &common::packet::Packet) -> Vec<(String, common::Value, Option<String>)> {
    use common::packet::Fact;
    let mut out: Vec<(String, common::Value, Option<String>)> = Vec::new();
    for (_, f) in p.facts() {
        let (name, value, unit) = match f {
            Fact::Sensed(r) => (
                r.quantity.label().to_string(),
                common::Value::Float(r.value),
                Some(r.unit.symbol().to_string()),
            ),
            Fact::Event(e) => (e.kind.label().to_string(), common::Value::Bool(e.on), None),
            _ => continue,
        };
        if out.iter().any(|(n, _, _)| *n == name) {
            continue;
        }
        out.push((name, value, unit));
    }
    if p.carrier.rssi_dbfs.is_finite() {
        out.push((
            "rssi_dbfs".into(),
            common::Value::Float((p.carrier.rssi_dbfs as f64 * 10.0).round() / 10.0),
            Some("dB".into()),
        ));
    }
    if p.carrier.snr_db.is_finite() {
        out.push((
            "snr_db".into(),
            common::Value::Float((p.carrier.snr_db as f64 * 10.0).round() / 10.0),
            Some("dB".into()),
        ));
    }
    out.push((
        "frequency_mhz".into(),
        common::Value::Float(p.carrier.center_hz as f64 / 1e6),
        Some("MHz".into()),
    ));
    out
}

/// The unit a field's name declares, and nothing more.
///
/// The decoders in this tree name a field after what it holds and what it is
/// in: `temperature_c`, `humidity_pct`, `wind_avg_km_h`. That convention is
/// the whole of the mapping, so a decoder added tomorrow gets its units right
/// by naming its fields the way the others do, and one that does not gets an
/// entity with no unit rather than a wrong one.
fn unit_of(name: &str) -> Option<String> {
    let suffix = |s: &str| name == s || name.ends_with(&format!("_{s}"));
    let unit = if suffix("c") {
        "\u{b0}C"
    } else if suffix("f") {
        "\u{b0}F"
    } else if suffix("pct") {
        "%"
    } else if suffix("hpa") {
        "hPa"
    } else if suffix("kpa") {
        "kPa"
    } else if suffix("psi") {
        "psi"
    } else if suffix("v") {
        "V"
    } else if suffix("a") {
        "A"
    } else if suffix("w") {
        "W"
    } else if suffix("kwh") {
        "kWh"
    } else if suffix("km_h") {
        "km/h"
    } else if suffix("m_s") {
        "m/s"
    } else if suffix("kt") {
        "kn"
    } else if suffix("mm") {
        "mm"
    } else if suffix("m") {
        "m"
    } else if suffix("ft") {
        "ft"
    } else if suffix("deg") {
        "\u{b0}"
    } else if suffix("ppm") {
        "ppm"
    } else if suffix("db") || suffix("dbfs") || suffix("dbm") {
        "dB"
    } else if suffix("hz") {
        "Hz"
    } else if suffix("mhz") {
        "MHz"
    } else if suffix("s") {
        "s"
    } else {
        return None;
    };
    Some(unit.into())
}

/// What Home Assistant plots a reading as, where its unit says.
///
/// Only where the class is certain from the unit and the name together: a
/// wrong device class is worse than none, since Home Assistant then refuses
/// the entity's unit or draws it on the wrong axis.
fn device_class(name: &str, unit: Option<&str>) -> Option<&'static str> {
    let has = |s: &str| {
        name == s || name.starts_with(&format!("{s}_")) || name.ends_with(&format!("_{s}"))
    };
    match unit? {
        "\u{b0}C" | "\u{b0}F" => Some("temperature"),
        "%" if has("humidity") || has("moisture") => Some("humidity"),
        "%" if has("battery") => Some("battery"),
        "hPa" | "kPa" | "psi" => Some("pressure"),
        "V" => Some("voltage"),
        "A" => Some("current"),
        "W" => Some("power"),
        "kWh" => Some("energy"),
        "km/h" | "m/s" | "kn" => Some("wind_speed"),
        "mm" if has("rain") || has("precipitation") => Some("precipitation"),
        "ppm" if has("co2") => Some("carbon_dioxide"),
        "dB" if has("rssi") => Some("signal_strength"),
        "Hz" | "MHz" => Some("frequency"),
        _ => None,
    }
}

/// The configuration message that makes one field an entity.
#[allow(clippy::too_many_arguments)]
fn discovery(
    broker: &Broker,
    node_id: &str,
    state_topic: &str,
    field: &str,
    unit: Option<&str>,
    space: &str,
    ident: &str,
    name: Option<&str>,
    vendor: Option<&str>,
) -> String {
    let mut config = serde_json::json!({
        "name": label(field),
        "unique_id": format!("{node_id}_{}", slug(field)),
        "object_id": format!("{node_id}_{}", slug(field)),
        "state_topic": state_topic,
        "value_template": format!("{{{{ value_json.{field} }}}}"),
        "availability_topic": broker.availability(),
        "expire_after": EXPIRE_AFTER_S,
        "device": {
            "identifiers": [node_id],
            "name": name.map(|n| format!("{n} ({ident})")).unwrap_or_else(|| format!("{} {ident}", space.to_uppercase())),
            // Who made the thing, where the decoder recovered it, and
            // otherwise what put it in the house. "unknown" was the honest
            // answer to a question nobody asked: the column is read to find
            // out where a device came from, and every one of these came from
            // here.
            "manufacturer": vendor.unwrap_or(HUB_NAME),
            "model": space.to_uppercase(),
            "via_device": HUB,
        },
    });
    if let Some(u) = unit {
        config["unit_of_measurement"] = serde_json::json!(u);
        // Every number here is a reading taken at a moment, which is what
        // makes it plottable and what keeps it out of the energy dashboard's
        // totals.
        config["state_class"] = serde_json::json!("measurement");
    }
    if let Some(c) = device_class(field, unit) {
        config["device_class"] = serde_json::json!(c);
    }
    config.to_string()
}

/// Every reading of one reception, as the one message the entities read.
fn state(readings: &[(String, common::Value, Option<String>)]) -> String {
    let mut map = serde_json::Map::new();
    for (name, value, _) in readings {
        let v = match value {
            common::Value::Int(i) => serde_json::json!(i),
            common::Value::Float(f) => serde_json::json!(f),
            common::Value::Bool(b) => serde_json::json!(b),
            common::Value::Text(t) => serde_json::json!(t),
        };
        map.insert(name.clone(), v);
    }
    serde_json::Value::Object(map).to_string()
}

/// A word Home Assistant will take in a topic and in an id.
fn slug(s: &str) -> String {
    let out: String = s
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '_' })
        .collect();
    let trimmed = out.trim_matches('_').to_string();
    if trimmed.is_empty() { "unnamed".into() } else { trimmed }
}

/// A field name as a person reads it: `temperature_c` is Temperature, since
/// the unit is on the entity and saying it twice reads as a stutter.
fn label(field: &str) -> String {
    let mut parts: Vec<&str> = field.split('_').collect();
    if parts.len() > 1 && unit_of(field).is_some() {
        parts.pop();
    }
    let mut out = parts.join(" ");
    if out.is_empty() {
        out = field.to_string();
    }
    // Words a person reads as letters rather than as a word. Without this a
    // level reads "Rssi", which is not how anybody says it.
    for word in ["rssi", "snr", "id", "crc", "uid", "mac", "co2", "pm"] {
        if out == word {
            return word.to_uppercase();
        }
    }
    let mut c = out.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => out,
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "homeassistant",
    summary: "Publish every device heard to Home Assistant over MQTT discovery",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(HomeAssistantNode::default()))
}

/// One MQTT packet out of the front of a buffer, for a test standing in
/// for a broker: its kind, its flags, its
/// body, and how much of the buffer it took.
pub fn mqtt_packet(buf: &[u8]) -> Option<(u8, u8, Vec<u8>, usize)> {
    if buf.len() < 2 {
        return None;
    }
    let (mut len, mut mult, mut i) = (0usize, 1usize, 1usize);
    loop {
        let b = *buf.get(i)?;
        len += (b & 127) as usize * mult;
        i += 1;
        if b & 128 == 0 {
            break;
        }
        mult *= 128;
    }
    if buf.len() < i + len {
        return None;
    }
    Some((buf[0] >> 4, buf[0] & 0x0f, buf[i..i + len].to_vec(), i + len))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use common::packet::Packet;

    fn packet(bytes: Vec<u8>, center_hz: u64) -> Packet {
        let mut p = crate::measured(center_hz, 2_000_000, bytes, -46.0, 20.0);
        p.carrier.at_us = 1_000_000;
        p
    }

    /// A Samsung monitor's BLE advertisement, dewhitened and CRC checked.
    fn advertisement() -> Packet {
        packet(
            vec![
                0x00, 0x11, 0x3a, 0xf5, 0x0a, 0xcd, 0x31, 0xe8, 0x02, 0x01, 0x06, 0x07, 0xff, 0xe1,
                0x02, 0x10, 0x00, 0x26, 0xc0,
            ],
            2_426_000_000,
        )
    }

    /// Through the protocols, the way the graph runs it.
    /// One block of speech through the node, as the audio bus delivers it.
    fn heard(node: &mut HomeAssistantNode, v: common::Voice) {
        feed(node, Payload::Packets(Vec::new()), Payload::Voice(vec![v]));
    }

    fn run(node: &mut HomeAssistantNode, packets: Vec<Packet>) {
        let mut packets = packets;
        crate::PacketDecodeNode::default().annotate(&mut packets);
        feed(node, Payload::Packets(packets), Payload::Voice(Vec::new()));
    }

    /// One block of each input, the way the graph hands them over: the
    /// packet bus and the tap.
    fn feed(node: &mut HomeAssistantNode, packets: Payload, voice: Payload) {
        let mut s = pipeline::StreamSpec::iq(0.0, Hz(2_426_000_000));
        s.kind = PortKind::Packets;
        let mut v = s;
        v.kind = PortKind::Voice;
        let ins = [PortSpec { spec: s, latency: 0 }, PortSpec { spec: v, latency: 0 }];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = [Payload::empty_of(PortKind::Real)];
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        ctx.block_seconds = 0.1;
        node.process(&[&packets, &voice], &mut out, &mut ctx).unwrap();
    }

    /// A block of speech as a fader's tap or a voice front end delivers it:
    /// who is talking, to whom, on what, and the audio.
    fn voice(system: &'static str, hz: f64, to: &str, from: Option<&str>, peak: f32) -> Payload {
        coded(system, hz, to, from, None, peak)
    }

    /// The same, with the coded squelch an analogue channel's users are set
    /// to.
    fn coded(
        system: &'static str,
        hz: f64,
        to: &str,
        from: Option<&str>,
        code: Option<&str>,
        peak: f32,
    ) -> Payload {
        Payload::Voice(vec![common::Voice {
            system,
            channel_hz: hz,
            to: Some(to.to_string()),
            from: from.map(str::to_string),
            code: code.map(str::to_string),
            over: None,
            rate: 8_000.0,
            channels: 1,
            pcm: vec![peak; 800],
        }])
    }

    /// Configuration messages the bridge, the call bus and the message bus
    /// cost, once per connection: the bridge's own connectivity entity, the
    /// call event, the on-air lamp, four last-heard sensors, the message
    /// event and the last message.
    const BUS_ANNOUNCEMENTS: u64 = 9;

    /// A node pointed at a broker that is not there. Nothing reaches a
    /// network: what is being tested is what would be said, not the saying.
    fn node() -> HomeAssistantNode {
        let p = Publisher::inert();
        p.set_broker(Some(Broker::new("broker.invalid")));
        HomeAssistantNode::with(p)
    }

    /// The whole path short of the broker: an advertisement off the bus
    /// becomes one device, its entities, and a reading.
    #[test]
    fn an_advertisement_becomes_a_device_and_its_readings() {
        let mut n = node();
        run(&mut n, vec![advertisement()]);
        assert_eq!(n.status().devices, 1);
        let known = n.known.values().next().expect("one device");
        // Level, noise ratio and the channel it was heard on: the three every
        // device has, whatever it decoded to.
        assert!(known.announced.contains("rssi_dbfs"), "{:?}", known.announced);
        assert!(known.announced.contains("snr_db"));
        assert!(known.announced.contains("frequency_mhz"));
    }

    /// A beacon advertising ten times a second is not ten messages a second.
    #[test]
    fn repeats_are_thinned_to_the_interval() {
        let mut n = node();
        run(&mut n, vec![advertisement(), advertisement(), advertisement()]);
        assert_eq!(n.status().devices, 1);
        // Three receptions, one publication of state and one announcement per
        // field. The publisher has no client, so every message is counted as
        // dropped rather than sent; what is being asserted is how many were
        // offered at all.
        let known = n.known.values().next().unwrap();
        let announced = known.announced.len() as u64;
        assert_eq!(n.publisher.status().dropped, announced + 1 + BUS_ANNOUNCEMENTS);
    }

    /// A burst nothing identified is not a device. The bus carries every
    /// unclaimed burst in the band, and a house does not want one entity per
    /// unknown pulse train.
    #[test]
    fn an_unidentified_burst_is_not_published() {
        let mut n = node();
        run(&mut n, vec![packet(vec![0x01, 0x02, 0x03], 433_920_000)]);
        assert_eq!(n.status().devices, 0);
        // The buses are announced whatever is on the air; the burst itself
        // said nothing.
        assert_eq!(n.publisher.status().dropped, BUS_ANNOUNCEMENTS);
    }

    /// Nothing is published at all until somebody has said where to.
    #[test]
    fn nothing_is_published_without_a_broker() {
        let mut n = HomeAssistantNode::with(Publisher::inert());
        assert!(!n.is_on());
        run(&mut n, vec![advertisement()]);
        assert_eq!(n.status().devices, 0);
    }

    /// Everything offered to the broker, topic and payload.
    fn said(n: &HomeAssistantNode) -> Vec<(String, String)> {
        n.publisher.said.lock().unwrap().clone()
    }

    fn payload<'a>(said: &'a [(String, String)], topic: &str) -> &'a str {
        said.iter().rev().find(|(t, _)| t == topic).map(|(_, p)| p.as_str()).unwrap_or_else(|| {
            let topics: Vec<&str> = said.iter().map(|(t, _)| t.as_str()).collect();
            panic!("nothing on {topic}, only {topics:?}")
        })
    }

    /// A block of a call, as a trunked system publishes one: the over says
    /// it is speech, in which vocoder and under what cipher, and the labels
    /// say who it is between.
    fn over(from: &str, to: &str) -> common::Voice {
        common::Voice {
            system: "TETRA",
            channel_hz: 391_035_600.0,
            to: Some(to.to_string()),
            from: Some(from.to_string()),
            code: None,
            over: Some(
                common::Over::new(Some("ACELP 4.6k"))
                    .protected_by(common::Secrecy::Clear)
                    .lasting(0.06),
            ),
            rate: 8_000.0,
            channels: 1,
            pcm: vec![0.4; 480],
        }
    }

    /// The whole point of the call bus: somebody keys up and the house can
    /// trigger on it, see who it was, and see the lamp go out afterwards.
    #[test]
    fn a_call_is_an_event_and_a_lamp() {
        let mut n = node();
        heard(&mut n, over("10223295", "Control 1"));
        let s = said(&n);

        let event: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/event")).unwrap();
        assert_eq!(event["event_type"], "call_started");
        assert_eq!(event["caller"], "10223295");
        assert_eq!(event["talkgroup"], "Control 1");
        assert_eq!(event["system"], "TETRA");
        assert_eq!(event["channel_mhz"], 391.0356);
        assert_eq!(event["encrypted"], false);
        assert_eq!(event["codec"], "ACELP 4.6k");

        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/state")).unwrap();
        assert_eq!(state["on_air"], "ON");
        assert_eq!(state["caller"], "10223295");

        // A second frame of the same over is the same call: an automation
        // that fired once a burst would fire fifty times a second.
        heard(&mut n, over("10223295", "Control 1"));
        let started = said(&n)
            .iter()
            .filter(|(t, p)| t == "waveshark/calls/event" && p.contains("call_started"))
            .count();
        assert_eq!(started, 1, "one over, one event");

        // And the call ends when nothing more is heard on it.
        n.age_calls(Instant::now() + Duration::from_secs_f64(CALL_HANG_S + 1.0));
        let s = said(&n);
        let event: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/event")).unwrap();
        assert_eq!(event["event_type"], "call_ended");
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/state")).unwrap();
        assert_eq!(state["on_air"], "OFF");
    }

    /// A call off the tap, which is the only place an analogue one exists.
    ///
    /// A receiver listening to a repeater on the strip runs no decoder, so
    /// it has no packet bus at all: fed only from packets, the call bus and
    /// the on-air lamp said nothing for the whole session.
    #[test]
    fn an_analogue_over_is_a_call_with_no_decoder_running() {
        let mut n = node();
        let quiet = Payload::Packets(Vec::new());

        // A squelched channel delivers blocks of nothing, and nothing is not
        // a call: the lamp would be lit for the session.
        feed(&mut n, quiet.clone(), voice("Audio", 145_500_000.0, "CH1", None, 0.0));
        assert!(
            !said(&n).iter().any(|(t, _)| t == "waveshark/calls/event"),
            "silence became a call"
        );

        // Somebody keys up on it.
        feed(&mut n, quiet.clone(), voice("Audio", 145_500_000.0, "CH1", None, 0.2));
        let s = said(&n);
        let event: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/event")).unwrap();
        assert_eq!(event["event_type"], "call_started");
        assert_eq!(event["talkgroup"], "CH1");
        assert_eq!(event["system"], "Audio");
        assert_eq!(event["channel_mhz"], 145.5);
        assert_eq!(event["encrypted"], false);
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/state")).unwrap();
        assert_eq!(state["on_air"], "ON");

        // More of the same over is the same call.
        feed(&mut n, quiet.clone(), voice("Audio", 145_500_000.0, "CH1", None, 0.2));
        let started = said(&n)
            .iter()
            .filter(|(t, p)| t == "waveshark/calls/event" && p.contains("call_started"))
            .count();
        assert_eq!(started, 1, "one over, one event");

        // And it ends on the hang, the way a decoded one does.
        n.age_calls(Instant::now() + Duration::from_secs_f64(CALL_HANG_S + 1.0));
        let s = said(&n);
        assert!(payload(&s, "waveshark/calls/event").contains("call_ended"));
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/state")).unwrap();
        assert_eq!(state["on_air"], "OFF");
    }

    /// The coded squelch reaches the house, so an automation can tell one
    /// group on a shared frequency from another.
    #[test]
    fn a_call_carries_the_coded_squelch_into_the_house() {
        let mut n = node();
        let quiet = Payload::Packets(Vec::new());
        // The over starts before the group has been read: half a second of
        // audio is what a tone or a code costs.
        feed(&mut n, quiet.clone(), coded("Audio", 446_049_100.0, "PMR1", None, None, 0.2));
        let s = said(&n);
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/state")).unwrap();
        assert_eq!(state["on_air"], "ON");
        assert_eq!(state["code"], "");

        // It lands, on the same call.
        feed(&mut n, quiet, coded("Audio", 446_049_100.0, "PMR1", None, Some("141.3"), 0.2));
        let s = said(&n);
        let started = s
            .iter()
            .filter(|(t, p)| t == "waveshark/calls/event" && p.contains("call_started"))
            .count();
        assert_eq!(started, 1, "the group made a second call");
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/state")).unwrap();
        assert_eq!(state["code"], "141.3");

        // And it is on the event that ends the call.
        n.age_calls(Instant::now() + Duration::from_secs_f64(CALL_HANG_S + 1.0));
        let s = said(&n);
        let event: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/event")).unwrap();
        assert_eq!(event["event_type"], "call_ended");
        assert_eq!(event["code"], "141.3");
    }

    /// A decoded call arrives twice, as audio on the tap and as frames on
    /// the bus, and is one call. Whichever half opens the row, the decode
    /// fills in what audio cannot say: the codec, and whether it was
    /// enciphered.
    #[test]
    fn one_call_heard_both_ways_is_one_call() {
        let mut n = node();
        let mut d = over("10223295", "Control 1");
        d.over.as_mut().unwrap().secrecy = common::Secrecy::Encrypted(None);
        // The audio first, with nobody named on it, which is the order a
        // vocoder delivers in.
        feed(
            &mut n,
            Payload::Packets(Vec::new()),
            voice("TETRA", 391_035_600.0, "Control 1", None, 0.2),
        );
        heard(&mut n, d);
        let started = said(&n)
            .iter()
            .filter(|(t, p)| t == "waveshark/calls/event" && p.contains("call_started"))
            .count();
        assert_eq!(started, 1, "the same call was published twice");
        let s = said(&n);
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/state")).unwrap();
        assert_eq!(state["caller"], "10223295", "the decode named who audio could not");
        n.age_calls(Instant::now() + Duration::from_secs_f64(CALL_HANG_S + 1.0));
        let s = said(&n);
        let event: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/calls/event")).unwrap();
        assert_eq!(event["event_type"], "call_ended");
        assert_eq!(event["encrypted"], true, "audio cannot say, and the decode did");
    }

    /// A block with no speech in it and nothing said about an over is not a
    /// call: a registration and a short data message both name parties.
    #[test]
    fn a_frame_that_is_not_speech_is_not_a_call() {
        let mut n = node();
        let mut d = over("10223295", "Control 1");
        d.over = None;
        d.pcm = vec![0.0; 480];
        heard(&mut n, d);
        assert!(
            !said(&n).iter().any(|(t, _)| t == "waveshark/calls/event"),
            "a data frame became a call"
        );
    }

    /// What somebody wrote reaches the house as an event and as the last
    /// message. The words are an attribute as well as the state, because a
    /// state is 255 characters and a page can be longer.
    #[test]
    fn a_message_is_an_event_and_the_last_message() {
        let mut n = node();
        let said_by = common::packet::Proto::new("tetra", "sds")
            .between(common::packet::Link::between(
                common::packet::Party::unit("10223295"),
                common::packet::Party::unit("15835885"),
            ))
            .saying(common::packet::Fact::message("rtb"));
        run(&mut n, vec![packet(vec![1], 391_035_600).decoded(said_by)]);
        let s = said(&n);
        let event: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/messages/event")).unwrap();
        assert_eq!(event["event_type"], "message");
        assert_eq!(event["text"], "rtb");
        assert_eq!(event["from"], "10223295");
        assert_eq!(event["to"], "15835885");
        assert_eq!(event["system"], "tetra");
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/messages/state")).unwrap();
        assert_eq!(state["text"], "rtb");
        assert_eq!(state["full_text"], "rtb");
    }

    /// A long page arrives whole in the attribute and cut in the state, since
    /// Home Assistant refuses a state longer than 255 characters.
    #[test]
    fn a_long_message_keeps_its_words_in_the_attribute() {
        let mut n = node();
        let long = "M".repeat(400);
        let page = common::packet::Proto::new("pocsag", "alpha")
            .saying(common::packet::Fact::message(long.clone()));
        run(&mut n, vec![packet(vec![1], 153_350_000).decoded(page)]);
        let s = said(&n);
        let state: serde_json::Value =
            serde_json::from_str(payload(&s, "waveshark/messages/state")).unwrap();
        assert_eq!(state["text"].as_str().unwrap().len(), STATE_MAX);
        assert_eq!(state["full_text"].as_str().unwrap(), long);
    }

    /// A house that wants its meters and not the radio traffic can have
    /// them: the buses are one switch, and it is off the same node.
    #[test]
    fn the_buses_can_be_turned_off() {
        let mut n = node();
        pipeline::node::Node::set_param(&mut n, "buses", pipeline::ParamValue::Bool(false))
            .unwrap();
        heard(&mut n, over("10223295", "Control 1"));
        let s = said(&n);
        assert!(!s.iter().any(|(t, _)| t.contains("calls")), "a call reached a broker");
        assert!(!s.iter().any(|(t, _)| t.contains("call_bus")), "the bus was announced");
    }

    /// Text nobody wrote is not a message here either: the same statement
    /// decides as in the message view.
    #[test]
    fn a_machine_talking_is_not_a_message() {
        let mut n = node();
        // What a station is playing is not a message: nobody wrote it and it
        // is addressed to nobody.
        let rds = common::packet::Proto::new("rds", "station")
            .saying(common::packet::Fact::Playing("NOW PLAYING".into()));
        run(&mut n, vec![packet(vec![1], 95_800_000).decoded(rds)]);
        assert!(!said(&n).iter().any(|(t, _)| t == "waveshark/messages/event"));
    }

    /// Every device says where it came from. `via_device` pointed at a bridge
    /// that was never announced, so Home Assistant dropped the link and a
    /// house full of discovered sensors read as coming from nobody.
    #[test]
    fn the_receiver_is_a_device_and_everything_is_published_through_it() {
        let mut n = node();
        run(&mut n, vec![advertisement()]);
        let s = said(&n);
        let hub: serde_json::Value = serde_json::from_str(payload(
            &s,
            "homeassistant/binary_sensor/waveshark/receiving/config",
        ))
        .unwrap();
        assert_eq!(hub["device"]["identifiers"][0], "waveshark");
        assert_eq!(hub["device"]["manufacturer"], "WaveShark");
        assert_eq!(hub["state_topic"], "waveshark/status");

        let device = "waveshark_ble_e8_31_cd_0a_f5_3a";
        let config: serde_json::Value = serde_json::from_str(payload(
            &s,
            &format!("homeassistant/sensor/{device}/rssi_dbfs/config"),
        ))
        .unwrap();
        assert_eq!(config["device"]["via_device"], "waveshark");
        // The maker where the decode named one.
        assert_eq!(config["device"]["manufacturer"], "Victron Energy");
        // And where it did not, who put it in the house rather than the word
        // "unknown", which is what every OOK sensor read as.
        let bare: serde_json::Value = serde_json::from_str(&discovery(
            &Broker::new("broker.invalid"),
            "waveshark_ism_43104",
            "waveshark/ism/43104/state",
            "temperature_c",
            Some("\u{b0}C"),
            "ism:Fineoffset-WHx080",
            "43104",
            None,
            None,
        ))
        .unwrap();
        assert_eq!(bare["device"]["manufacturer"], "WaveShark");

        let bus: serde_json::Value =
            serde_json::from_str(payload(&s, "homeassistant/event/waveshark_call_bus/call/config"))
                .unwrap();
        assert_eq!(bus["device"]["via_device"], "waveshark");
        assert_eq!(bus["event_types"][0], "call_started");
        assert_eq!(bus["event_types"][1], "call_ended");
    }

    /// The units come off the field names, which is the convention every
    /// decoder in the tree already follows.
    #[test]
    fn a_field_name_carries_its_own_unit() {
        assert_eq!(unit_of("temperature_c").as_deref(), Some("\u{b0}C"));
        assert_eq!(unit_of("humidity_pct").as_deref(), Some("%"));
        assert_eq!(unit_of("wind_avg_km_h").as_deref(), Some("km/h"));
        assert_eq!(unit_of("battery_v").as_deref(), Some("V"));
        assert_eq!(unit_of("pressure_hpa").as_deref(), Some("hPa"));
        assert_eq!(unit_of("altitude_ft").as_deref(), Some("ft"));
        // A name that says nothing about its unit gets none rather than a
        // guess: a wrong unit is worse than a bare number.
        assert_eq!(unit_of("id"), None);
        assert_eq!(unit_of("tristate"), None);

        assert_eq!(device_class("temperature_c", Some("\u{b0}C")), Some("temperature"));
        assert_eq!(device_class("humidity_pct", Some("%")), Some("humidity"));
        // A percentage that is not humidity or charge is a percentage.
        assert_eq!(device_class("duty_pct", Some("%")), None);
        assert_eq!(device_class("rssi_dbfs", Some("dB")), Some("signal_strength"));
        assert_eq!(device_class("snr_db", Some("dB")), None);
    }

    /// What a person reads on the entity, with the unit taken off the end.
    #[test]
    fn an_entity_is_named_for_what_it_holds() {
        assert_eq!(label("temperature_c"), "Temperature");
        assert_eq!(label("wind_avg_km_h"), "Wind avg km");
        assert_eq!(label("battery_ok"), "Battery ok");
        // Letters, not words.
        assert_eq!(label("rssi_dbfs"), "RSSI");
        assert_eq!(label("snr_db"), "SNR");
        assert_eq!(label("id"), "ID");
    }

    /// A house can ask for its own meters and not the street's handsets.
    ///
    /// The reason the filter exists: a resolvable-private BLE address rotates
    /// every quarter of an hour, so a receiver left running would publish a
    /// new device four times an hour for every phone that walks past, and
    /// Home Assistant keeps every one it is told about.
    #[test]
    fn a_space_filter_keeps_the_street_out_of_the_house() {
        let mut n = node();
        n.set_spaces("ism,wmbus");
        run(&mut n, vec![advertisement()]);
        assert_eq!(n.status().devices, 0);

        n.set_spaces("ble");
        run(&mut n, vec![advertisement()]);
        assert_eq!(n.status().devices, 1);

        // Empty is everything, which is what a receiver starts as.
        let mut n = node();
        n.set_spaces("");
        run(&mut n, vec![advertisement()]);
        assert_eq!(n.status().devices, 1);
    }

    /// Ids and topics Home Assistant will take, from identifiers it will not.
    #[test]
    fn an_address_becomes_something_a_topic_can_hold() {
        assert_eq!(slug("E8:31:CD:0A:F5:3A"), "e8_31_cd_0a_f5_3a");
        assert_eq!(slug("temperature_c"), "temperature_c");
        assert_eq!(slug("!@#"), "unnamed");
    }

    /// One state message carries every reading, which is what the entities'
    /// templates read.
    #[test]
    fn one_message_carries_every_reading() {
        let readings = vec![
            ("temperature_c".into(), common::Value::Float(16.2), Some("\u{b0}C".into())),
            ("humidity_pct".into(), common::Value::Int(89), Some("%".into())),
            ("id".into(), common::Value::Text("43104".into()), None),
        ];
        let s = state(&readings);
        assert!(s.contains("\"temperature_c\":16.2"), "{s}");
        assert!(s.contains("\"humidity_pct\":89"), "{s}");
        assert!(s.contains("\"id\":\"43104\""), "{s}");
    }

    /// A weather station, as the ISM decoders name one: `ism:<model>` and
    /// the id printed on it. What matters is that its fields arrive as
    /// entities Home Assistant plots, with the units and classes the names
    /// imply, rather than as a line of text.
    #[test]
    fn a_sensor_becomes_the_entities_a_house_plots() {
        let r = decode::Report::new("Fineoffset-WHx080")
            .int("id", 199)
            .float("temperature_c", 16.2)
            .int("humidity_pct", 89)
            .bool("battery_ok", true);
        let p = packet(vec![0xab, 0xcd], 433_920_000).decoded(decode::facts::proto_of(&r));

        let mut n = node();
        n.set_spaces("ism");
        run(&mut n, vec![p]);
        assert_eq!(n.status().devices, 1);
        let known = n.known.values().next().unwrap();
        // The numbers become entities; the model name and the flag ride
        // along in the state message without one, since a house does not
        // want a sensor whose value is a model number.
        // Named for what was measured rather than for the field it came in:
        // a chart keys on the quantity.
        assert!(known.announced.contains("temperature"), "{:?}", known.announced);
        assert!(known.announced.contains("humidity"));
        assert!(!known.announced.contains("model"), "{:?}", known.announced);
        assert_eq!(known.announced.len(), 5);

        let broker = Broker::new("broker.invalid");
        let config = discovery(
            &broker,
            "waveshark_ism_fineoffset_whx080_199",
            "waveshark/ism_fineoffset_whx080/199/state",
            "temperature_c",
            unit_of("temperature_c").as_deref(),
            "ism:Fineoffset-WHx080",
            "199",
            None,
            None,
        );
        assert!(config.contains("\"device_class\":\"temperature\""), "{config}");
        assert!(config.contains("\"unit_of_measurement\":\"\u{b0}C\""), "{config}");
        assert!(config.contains("\"state_class\":\"measurement\""), "{config}");
        assert!(
            config.contains("\"value_template\":\"{{ value_json.temperature_c }}\""),
            "{config}"
        );
    }

    /// A device named by a later frame is announced again with its name.
    ///
    /// The name lives in the device block of each entity's configuration
    /// and nowhere else, and the configurations went out once, on the
    /// first frame, which for a BLE device is an advertisement without one.
    /// The house then showed an address forever, whatever the device later
    /// said it was called.
    #[test]
    fn a_name_learned_later_is_announced() {
        let ble = |name: Option<&str>| {
            let mut who =
                common::packet::Entity::new("ble", common::packet::Id::Text("aa:bb".into()));
            who.name = name.map(str::to_string);
            packet(vec![1], 2_426_000_000).decoded(common::packet::Proto::new("ble", "adv").by(who))
        };
        let first = ble(None);
        let mut n = node();
        n.min_interval = Duration::ZERO;
        run(&mut n, vec![first]);
        let before = n.known.values().next().unwrap().announced.len();
        assert!(before > 0);
        assert_eq!(n.known.values().next().unwrap().named, (None, None));

        run(&mut n, vec![ble(Some("Kitchen scale"))]);
        let k = n.known.values().next().unwrap();
        assert_eq!(k.named.0.as_deref(), Some("Kitchen scale"));
        assert_eq!(k.announced.len(), before, "announced again, the same fields");
        assert_eq!(n.status().devices, 1, "the same device, not a second one");

        // A frame without the name does not unlearn it.
        run(&mut n, vec![ble(None)]);
        assert_eq!(n.known.values().next().unwrap().named.0.as_deref(), Some("Kitchen scale"));
    }

    /// The network leg, against a socket that speaks just enough MQTT to
    /// accept a connection and read what arrives.
    ///
    /// Everything above is arithmetic on decodes; this is the only test that
    /// says the client connects at all, announces itself, and puts a device's
    /// configuration and its readings on the topics Home Assistant is
    /// listening to.
    #[test]
    fn a_device_reaches_the_broker() {
        use std::io::Read;
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("a port");
        let port = listener.local_addr().unwrap().port();
        let (tx, rx) = std::sync::mpsc::channel::<(String, String)>();
        std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().expect("a connection");
            let mut buf = Vec::new();
            let mut chunk = [0u8; 4096];
            loop {
                let n = match sock.read(&mut chunk) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                buf.extend_from_slice(&chunk[..n]);
                while let Some((kind, flags, body, used)) = take_packet(&buf) {
                    buf.drain(..used);
                    match kind {
                        // CONNECT, answered so the client believes it is up.
                        1 => {
                            use std::io::Write;
                            if sock.write_all(&[0x20, 0x02, 0x00, 0x00]).is_err() {
                                return;
                            }
                        }
                        3 => {
                            let tl = u16::from_be_bytes([body[0], body[1]]) as usize;
                            let topic = String::from_utf8_lossy(&body[2..2 + tl]).to_string();
                            // A published packet above QoS 0 carries an
                            // identifier between the topic and the payload.
                            let qos = (flags >> 1) & 3;
                            let at = 2 + tl + if qos > 0 { 2 } else { 0 };
                            let payload = String::from_utf8_lossy(&body[at..]).to_string();
                            if tx.send((topic, payload)).is_err() {
                                return;
                            }
                        }
                        _ => {}
                    }
                }
            }
        });

        let publisher = Publisher::inert();
        let broker = Broker { port, ..Broker::new("127.0.0.1") };
        publisher.set_broker(Some(broker));
        let p = publisher.clone();
        std::thread::spawn(move || {
            let b = p.broker().unwrap();
            let _ = p.connect(&b);
        });
        // The connection has to be up before anything is published: a
        // message handed to a client that is not connected is a dropped
        // reading, which is the behaviour the radio thread wants and the
        // wrong thing to assert here.
        let up = Instant::now();
        while !publisher.is_connected() && up.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(publisher.is_connected(), "the client never connected");

        let mut node = HomeAssistantNode::with(publisher.clone());
        run(&mut node, vec![advertisement()]);
        assert_eq!(node.status().devices, 1);

        let mut seen: Vec<(String, String)> = Vec::new();
        while let Ok(msg) = rx.recv_timeout(Duration::from_secs(2)) {
            seen.push(msg);
            // Availability, the bus announcements, one configuration per
            // announced field, and the one state message they all read.
            let want =
                node.known.values().next().unwrap().announced.len() as u64 + 2 + BUS_ANNOUNCEMENTS;
            if seen.len() as u64 >= want {
                break;
            }
        }
        let topics: Vec<&str> = seen.iter().map(|(t, _)| t.as_str()).collect();
        assert_eq!(topics.first().copied(), Some("waveshark/status"));
        assert_eq!(seen[0].1, "online");
        let device = "waveshark_ble_e8_31_cd_0a_f5_3a";
        let config = seen
            .iter()
            .find(|(t, _)| t == &format!("homeassistant/sensor/{device}/rssi_dbfs/config"))
            .unwrap_or_else(|| panic!("no configuration for the level in {topics:?}"));
        assert!(config.1.contains("\"device_class\":\"signal_strength\""), "{}", config.1);
        assert!(
            config.1.contains("\"state_topic\":\"waveshark/ble/e8_31_cd_0a_f5_3a/state\""),
            "{}",
            config.1
        );
        let state = seen
            .iter()
            .find(|(t, _)| t == "waveshark/ble/e8_31_cd_0a_f5_3a/state")
            .unwrap_or_else(|| panic!("no reading in {topics:?}"));
        assert!(state.1.contains("\"rssi_dbfs\":-46"), "{}", state.1);
        assert_eq!(publisher.status().dropped, 0);
    }

    use super::mqtt_packet as take_packet;
}
