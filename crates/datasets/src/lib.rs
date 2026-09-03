//! Reference data the receiver reads but does not produce: airports and
//! their frequencies, the DMR and NXDN registries, the gateways of the
//! digital voice networks, and whatever comes next.
//!
//! None of it is signal, so none of it belongs in the flow graph. What it
//! shares is a lifecycle: published by somebody else, large enough to be
//! worth keeping, and stale eventually. [`cache`] is that lifecycle, and each
//! module here is one dataset expressed in terms of it: a [`cache::Source`]
//! saying where the file comes from, a parse, and a refresh that reparses
//! only when the file actually changed.

pub mod airports;
pub mod cache;
pub mod gateways;
pub mod m17;
pub mod pistar;
pub mod radioid;

pub use cache::{Cache, Error, Source, Status, When};
