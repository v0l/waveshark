//! Tuners, and many network readers of each.
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
//! # Getting to a client the server cannot address
//!
//! A subscriber behind NAT cannot say where it is, so it punches: a datagram
//! to the server's data port carrying [`proto::tag::PUNCH_TOKEN`], and the
//! server sends the samples back to the address that arrived on. The size of
//! those datagrams is measured rather than assumed, with probes the client
//! answers only for the sizes that reached it, because a 1500 byte datagram
//! is dropped outright on an ordinary PPPoE path.
//!
//! Where no datagram arrives at all the client asks again for
//! [`proto::Transport::Tcp`] and reads the samples off the control
//! connection. That costs head of line blocking, so a subscriber not taking
//! them has its blocks dropped rather than its keepalives held up.
//!
//! docs/iqstream-nat.md draws the whole exchange.
//!
//! # Several tuners on one port
//!
//! A server offers a [`server::Stream`] per tuner and says so in its welcome;
//! a subscriber names the one it wants, and a tune names the dial it means. A
//! 1.1 peer names none, and a server hands it the first, which is the only
//! stream such a server ever had.
//!
//! What a tuner is set to travels with it: its gain stages, its switches and
//! its antenna port are in its description, and moving any of them sends
//! every connection a fresh one. A reader that is not told cannot say what
//! level its samples were heard at.
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

pub use client::{Block, ClientConfig, IqStream, Prefer, StreamInfo, list};
pub use proto::{Codec, Setting, SettingKind, SettingValue, StreamDesc, Transport};
pub use server::{Ask, Public, Server, ServerConfig, Stream, StreamConfig, Tune};
