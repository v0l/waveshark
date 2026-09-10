//! Front ends that take cipher keys, asked as a capability rather than by
//! name.
//!
//! The key manager reaches every front end reading an encrypted system,
//! wherever it sits: placed by the scanner table, drawn by hand, built by
//! the auto node for a source it found a moment ago, or running on a channel
//! inside a bank. It asks each node in the receiver whether it is keyed, so
//! nothing here or in the receiver keeps a list of which kinds are.

use crate::tetra_nodes::{KeyStatus, TetraNode};
use pipeline::node::Node;

/// A front end reading a system whose traffic is enciphered per cell colour.
pub trait Keyed {
    /// What the key manager shows for the cell this front end is on, or
    /// `None` until it has read one.
    fn key_status(&self) -> Option<KeyStatus>;

    /// Give this colour code's traffic a key, so it decodes.
    #[cfg(feature = "tea")]
    fn add_key(&mut self, colour: u8, key: decode::tea::Key);

    /// Give this colour code an identity secret, so its encrypted identities
    /// show as the real subscribers.
    #[cfg(feature = "tea")]
    fn add_id_secret(&mut self, colour: u8, c: [u8; 8]);
}

impl Keyed for TetraNode {
    fn key_status(&self) -> Option<KeyStatus> {
        TetraNode::key_status(self)
    }

    #[cfg(feature = "tea")]
    fn add_key(&mut self, colour: u8, key: decode::tea::Key) {
        TetraNode::add_key(self, colour, key)
    }

    #[cfg(feature = "tea")]
    fn add_id_secret(&mut self, colour: u8, c: [u8; 8]) {
        TetraNode::add_id_secret(self, colour, c)
    }
}

/// The keyed view of a node, or `None` where it takes no keys.
///
/// The one place that knows which kinds of node are keyed, so a caller can
/// ask every node in the graph and take the answer. A free function rather
/// than a method on [`Node`] because the graph is a layer below this one and
/// cannot name a cipher key or a cell.
pub fn keyed(n: &dyn Node) -> Option<&dyn Keyed> {
    Some(n.as_any().downcast_ref::<TetraNode>()?)
}

/// The mutable counterpart of [`keyed`].
pub fn keyed_mut(n: &mut dyn Node) -> Option<&mut dyn Keyed> {
    Some(n.as_any_mut().downcast_mut::<TetraNode>()?)
}
