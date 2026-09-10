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

use common::Result;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
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
        if self.prefix.trim().is_empty() {
            "homeassistant"
        } else {
            self.prefix.trim()
        }
    }

    fn topic(&self) -> &str {
        if self.topic.trim().is_empty() {
            "waveshark"
        } else {
            self.topic.trim()
        }
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
        })
    }

    /// The one publisher, started the first time a node asks for it.
    pub fn shared() -> Arc<Self> {
        static PUBLISHER: OnceLock<Arc<Publisher>> = OnceLock::new();
        let p = PUBLISHER.get_or_init(Publisher::inert);
        p.start();
        p.clone()
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
    }

    pub fn broker(&self) -> Option<Broker> {
        self.broker.lock().ok().and_then(|b| b.clone())
    }

    fn disconnect(&self) {
        self.connected.store(false, Ordering::Relaxed);
        if let Ok(mut c) = self.client.lock() {
            if let Some(client) = c.take() {
                let _ = client.try_disconnect();
            }
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
                std::thread::sleep(RETRY);
                continue;
            };
            match self.connect(&broker) {
                Ok(()) => wait = RETRY,
                Err(e) => {
                    if let Ok(mut held) = self.error.lock() {
                        *held = Some(e);
                    }
                    std::thread::sleep(wait);
                    wait = (wait * 2).min(RETRY_MAX);
                }
            }
            self.connected.store(false, Ordering::Relaxed);
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
    /// The connection those messages were sent over. A broker that restarted
    /// has lost them, and its generation says so.
    generation: u64,
    last: Instant,
}

/// The feed to Home Assistant, on the packet bus.
pub struct HomeAssistantNode {
    publisher: Arc<Publisher>,
    known: HashMap<(String, String), Known>,
    devices: u64,
    /// The least time between two publications about one device.
    min_interval: Duration,
    max_devices: usize,
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
    fn default() -> Self {
        Self::with(Publisher::shared())
    }
}

impl HomeAssistantNode {
    pub fn with(publisher: Arc<Publisher>) -> Self {
        Self {
            publisher,
            known: HashMap::new(),
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

    /// Which identity spaces to publish, as a comma-separated list, or empty
    /// for every one of them.
    pub fn set_spaces(&mut self, spaces: &str) {
        self.spaces =
            spaces.split(',').map(|s| s.trim().to_lowercase()).filter(|s| !s.is_empty()).collect();
    }

    pub fn is_on(&self) -> bool {
        self.publisher.broker().is_some()
    }

    pub fn status(&self) -> HomeAssistantStatus {
        HomeAssistantStatus { devices: self.devices, ..self.publisher.status() }
    }

    /// One decode, as a device in a house.
    fn publish(&mut self, p: &common::Packet, d: &common::Decoded, now: Instant) {
        let Some((space, ident)) = crate::survey_nodes::identity(d) else { return };
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

        let readings = readings(p, d);
        let entry = self.known.entry(key).or_insert_with(|| Known {
            announced: HashSet::new(),
            generation,
            last: now,
        });
        // A broker that restarted has forgotten every retained configuration,
        // so a new connection is a new introduction.
        if entry.generation != generation {
            entry.announced.clear();
            entry.generation = generation;
        }
        let fresh = entry.announced.is_empty();
        for (name, value, unit) in &readings {
            if entry.announced.len() >= MAX_FIELDS || entry.announced.contains(name) {
                continue;
            }
            if !matches!(value, common::Value::Float(_) | common::Value::Int(_)) {
                continue;
            }
            entry.announced.insert(name.clone());
            let config = discovery(
                &broker,
                &node_id,
                &state_topic,
                name,
                unit.as_deref(),
                &space,
                &ident,
                crate::survey_nodes::name_of(d).as_deref(),
                crate::survey_nodes::vendor_of(d).as_deref(),
            );
            self.publisher.send(
                &format!("{}/sensor/{node_id}/{}/config", broker.prefix(), slug(name)),
                config,
                true,
            );
        }
        if fresh {
            self.devices += 1;
        }
        entry.last = now;
        self.publisher.send(&state_topic, state(&readings), true);
    }
}

impl Simple for HomeAssistantNode {
    fn name(&self) -> &str {
        "homeassistant"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("homeassistant reads the packet bus"));
        }
        Ok(i.spec)
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
            _ => Err(common::Error::other(format!("no parameter {name}"))),
        }
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if !self.is_on() {
            return Ok(());
        }
        let now = Instant::now();
        for p in i.as_packets().unwrap_or(&[]) {
            for d in p.decodes.iter() {
                self.publish(p, d, now);
            }
        }
        Ok(())
    }
}

/// What is worth publishing about one reception: the decoder's own fields,
/// and how strongly it was heard.
///
/// The level is here rather than left out because it is the one reading every
/// device has, and it is what says a sensor is going out of range before it
/// stops reporting altogether.
fn readings(
    p: &common::Packet,
    d: &common::Decoded,
) -> Vec<(String, common::Value, Option<String>)> {
    let mut out: Vec<(String, common::Value, Option<String>)> = Vec::new();
    for (name, value) in &d.fields {
        if name.is_empty() || out.iter().any(|(n, _, _)| n == name) {
            continue;
        }
        out.push((name.clone(), value.clone(), unit_of(name)));
    }
    if p.rssi_dbfs().is_finite() {
        out.push((
            "rssi_dbfs".into(),
            common::Value::Float((p.rssi_dbfs() as f64 * 10.0).round() / 10.0),
            Some("dB".into()),
        ));
    }
    if p.snr_db().is_finite() {
        out.push((
            "snr_db".into(),
            common::Value::Float((p.snr_db() as f64 * 10.0).round() / 10.0),
            Some("dB".into()),
        ));
    }
    out.push((
        "frequency_mhz".into(),
        common::Value::Float(d.center.as_f64() / 1e6),
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
            "manufacturer": vendor.unwrap_or("unknown"),
            "model": space.to_uppercase(),
            "via_device": "waveshark",
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
    if trimmed.is_empty() {
        "unnamed".into()
    } else {
        trimmed
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Hz, Packet};

    fn packet(bytes: Vec<u8>, center_hz: u64) -> Packet {
        Packet::of_frame(
            1_000_000,
            2_000_000,
            common::Frame::measured(bytes, -46.0, 20.0).at(center_hz),
        )
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
    fn run(node: &mut HomeAssistantNode, packets: Vec<Packet>) {
        let mut packets = packets;
        crate::PacketDecodeNode::default().annotate(&mut packets);
        let mut s = pipeline::StreamSpec::iq(0.0, Hz(2_426_000_000));
        s.kind = PortKind::Packets;
        let ins = [PortSpec { spec: s, latency: 0 }];
        let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut out = Payload::Packets(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        node.process(&Payload::Packets(packets), &mut out, &mut ctx).unwrap();
    }

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
        assert_eq!(n.publisher.status().dropped, announced + 1);
    }

    /// A burst nothing identified is not a device. The bus carries every
    /// unclaimed burst in the band, and a house does not want one entity per
    /// unknown pulse train.
    #[test]
    fn an_unidentified_burst_is_not_published() {
        let mut n = node();
        run(&mut n, vec![packet(vec![0x01, 0x02, 0x03], 433_920_000)]);
        assert_eq!(n.status().devices, 0);
        assert_eq!(n.publisher.status().dropped, 0);
    }

    /// Nothing is published at all until somebody has said where to.
    #[test]
    fn nothing_is_published_without_a_broker() {
        let mut n = HomeAssistantNode::with(Publisher::inert());
        assert!(!n.is_on());
        run(&mut n, vec![advertisement()]);
        assert_eq!(n.status().devices, 0);
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
        let mut p = packet(vec![0xab, 0xcd], 433_920_000);
        let mut d = common::Decoded::bytes("ism", common::Hz(433_920_000), 0.0, vec![0xab, 0xcd]);
        d.fields = vec![
            ("temperature_c".into(), common::Value::Float(16.2)),
            ("humidity_pct".into(), common::Value::Int(89)),
            ("battery_ok".into(), common::Value::Bool(true)),
            ("model".into(), common::Value::Text("Fineoffset-WHx080".into())),
        ];
        d.identity = Some(common::Identity::new("ism:Fineoffset-WHx080", "199"));
        p.decodes.push(d);

        let mut n = node();
        n.set_spaces("ism");
        run(&mut n, vec![p]);
        assert_eq!(n.status().devices, 1);
        let known = n.known.values().next().unwrap();
        // The numbers become entities; the model name and the flag ride
        // along in the state message without one, since a house does not
        // want a sensor whose value is a model number.
        assert!(known.announced.contains("temperature_c"), "{:?}", known.announced);
        assert!(known.announced.contains("humidity_pct"));
        assert!(!known.announced.contains("model"), "{:?}", known.announced);
        assert!(!known.announced.contains("battery_ok"));
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
            // Availability, one configuration per announced field, and the
            // one state message they all read.
            if seen.len() >= node.known.values().next().unwrap().announced.len() + 2 {
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

    /// One MQTT packet out of the front of a buffer: its kind, its flags, its
    /// body, and how much of the buffer it took.
    fn take_packet(buf: &[u8]) -> Option<(u8, u8, Vec<u8>, usize)> {
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
}
