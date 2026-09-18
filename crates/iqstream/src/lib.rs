//! One tuner, many network readers.
//!
//! [`proto`] is the wire format, vendored from the reference implementation so
//! the receiver can speak both ends of it. [`client`] subscribes to somebody
//! else's tuner; [`server`] hands this receiver's span out to subscribers.
//!
//! Control is TCP frames of TLVs, samples are UDP datagrams, and a subscriber
//! chooses its own bit depth and compression. The control connection is the
//! subscription's lifetime: a reader that stops answering keepalives is
//! dropped.
//!
//! # Moving the dial from the other end
//!
//! A server says in its welcome whether it will be tuned ([`proto::tag::TUNABLE`]),
//! and a subscriber asks with [`proto::msg::TUNE`]. Where it lands comes back
//! to every subscriber as [`proto::msg::TUNED`], not only to the one that
//! asked, because the others are now reading a different piece of spectrum
//! than the one they subscribed to and nothing else would tell them.

pub mod client;
pub mod proto;
pub mod server;

pub use client::{Block, ClientConfig, IqStream, StreamInfo};
pub use proto::Codec;
pub use server::{Server, ServerConfig, Tune};
