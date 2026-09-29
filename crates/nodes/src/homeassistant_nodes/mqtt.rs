use super::Broker;
use common::time::Duration;

pub(super) const CONNECTS: bool = true;

#[derive(Clone)]
pub(super) struct Client(rumqttc::Client);

impl Client {
    pub(super) fn publish(&self, topic: &str, retain: bool, sure: bool, payload: String) -> bool {
        let qos = match sure {
            true => rumqttc::QoS::AtLeastOnce,
            false => rumqttc::QoS::AtMostOnce,
        };
        self.0.try_publish(topic, qos, retain, payload).is_ok()
    }

    pub(super) fn disconnect(&self) {
        let _ = self.0.try_disconnect();
    }
}

pub(super) struct Connection(rumqttc::Connection);

impl Connection {
    pub(super) fn accepted(&mut self) -> Option<Result<bool, String>> {
        let event = self.0.iter().next()?;
        Some(match event {
            Ok(rumqttc::Event::Incoming(rumqttc::Packet::ConnAck(ack))) => {
                match ack.code == rumqttc::ConnectReturnCode::Success {
                    true => Ok(true),
                    false => Err(format!("the broker refused the connection: {:?}", ack.code)),
                }
            }
            Ok(_) => Ok(false),
            Err(e) => Err(e.to_string()),
        })
    }
}

pub(super) fn open(id: String, broker: &Broker, queue: usize) -> (Client, Connection) {
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
    let (client, conn) = rumqttc::Client::new(opts, queue);
    (Client(client), Connection(conn))
}
