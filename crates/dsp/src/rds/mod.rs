//! RDS: the 57 kHz data subcarrier on FM broadcast.

pub mod block;
pub mod demod;
pub mod group;
pub mod tx;

pub use block::{BlockSync, Group, Offset};
pub use demod::RdsDemod;
pub use group::{GroupDecoder, Station};
