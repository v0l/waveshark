//! The waveshark flow graph.
//!
//! A DAG of typed, rate-negotiated nodes, in the spirit of GNU Radio but with
//! three deliberate departures:
//!
//! 1. **No thread per node.** A graph runs serially on one thread; parallelism
//!    comes from running many independent per-channel graphs across a pool.
//!    See [`graph`] for why.
//! 2. **No `forecast`/`consume`/`produce`.** A node is handed what arrived and
//!    emits what it can, buffering internally if it needs more.
//! 3. **No dynamically typed metadata.** Tags and events are Rust enums.
//!
//! Stream tags and a separate async event path are kept, because both are
//! things GNU Radio got right.

pub mod event;
pub mod graph;
pub mod node;
pub mod param;
pub mod port;
pub mod registry;

pub use event::{Decoded, Event};
pub use graph::{chain, Graph, GraphBuilder, In, NodeId, NodePart, Out, Topology, GRAPH_INPUT};
pub use node::{Node, NodeCtx, PortSpec, Simple};
pub use param::{Param, ParamRange, ParamValue};
pub use registry::Registry as NodeRegistry;
pub use port::{Payload, PortKind, StreamSpec, Tag, TagValue};
pub use registry::{Registry, Settings, SettingsExt, StageDesc};
