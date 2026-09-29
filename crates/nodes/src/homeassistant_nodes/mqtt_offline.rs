use super::Broker;

pub(super) const CONNECTS: bool = false;

#[derive(Clone)]
pub(super) struct Client;

impl Client {
    pub(super) fn publish(&self, _: &str, _: bool, _: bool, _: String) -> bool {
        false
    }

    pub(super) fn disconnect(&self) {}
}

pub(super) struct Connection(bool);

impl Connection {
    pub(super) fn accepted(&mut self) -> Option<Result<bool, String>> {
        match std::mem::replace(&mut self.0, true) {
            true => None,
            false => Some(Err("this build has no MQTT".into())),
        }
    }
}

pub(super) fn open(_: String, _: &Broker, _: usize) -> (Client, Connection) {
    (Client, Connection(false))
}
