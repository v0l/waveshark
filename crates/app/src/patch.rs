//! A graph the operator drew, rather than one the scanner table implied.
//!
//! In manual mode the front ends are no longer derived from where the dial is:
//! they are whatever is in here. That makes a patch the answer to a question
//! the automatic path never has to ask, which is what the operator meant, as
//! opposed to what happens to be running. The receiver rebuilds several times
//! a minute (a retune, a parameter that changes a rate, a channel opening) and
//! a description that lived only in the built graph would be lost at the first
//! one.
//!
//! Stages are named by an id of this module's own making, not by `NodeId`: a
//! `NodeId` is a position in the built graph and every rebuild renumbers them.
//! The id is carried into the graph as a tag so the view can match a box on
//! screen to the stage that asked for it.

use pipeline::registry::Settings;

/// The receiver's own stages, as link targets.
///
/// The spectrum and the recorder read the head of the chain unless the
/// operator says otherwise, and saying otherwise is the whole reason to put a
/// stage between the two. They are named by reserved ids rather than by a
/// second kind of link so that a wire is a wire: the view, the patch and the
/// builder all treat them like any other target.
pub mod builtin {
    /// The spectrum behind the waterfall.
    pub const SPECTRUM: u64 = u64::MAX;
    /// The recorder's ring.
    pub const RECORDER: u64 = u64::MAX - 1;
    /// The span itself: the samples every branch reads. Not a stage, but it
    /// is a box on screen and it is dragged and wired like one.
    pub const SPAN: u64 = u64::MAX - 2;

    /// Ids at or above this belong to the receiver rather than to the patch.
    pub const FIRST: u64 = u64::MAX - 15;

    pub fn is(id: u64) -> bool {
        id >= FIRST
    }
}

/// Where a link starts.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Source {
    /// The receiver's own samples, after the DC block and the zoom decimator:
    /// the same stream every automatic branch reads.
    Span,
    /// An output port of another stage in this patch.
    Stage(u64, usize),
}

/// One stage, as asked for rather than as built.
#[derive(Clone, PartialEq, Debug)]
pub struct Stage {
    pub id: u64,
    /// A registry name, such as "mixer" or "protocol_decode".
    pub kind: String,
    /// What it is constructed with. Only read when the node is first built:
    /// afterwards the node itself holds its parameters, and it survives a
    /// rebuild rather than being made again.
    pub settings: Settings,
}

/// One wire.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Link {
    pub from: Source,
    /// The stage and input port being fed.
    pub to: (u64, usize),
}

#[derive(Clone, Default, PartialEq, Debug)]
pub struct Patch {
    stages: Vec<Stage>,
    links: Vec<Link>,
    /// Ids are never reused inside one patch, so a stage deleted and another
    /// added cannot inherit its wires or its position.
    next: u64,
}

impl Patch {
    pub fn stages(&self) -> &[Stage] {
        &self.stages
    }

    /// For setting what a stage is built with before it is built.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn stages_mut(&mut self) -> &mut [Stage] {
        &mut self.stages
    }

    pub fn links(&self) -> &[Link] {
        &self.links
    }

    pub fn stage(&self, id: u64) -> Option<&Stage> {
        self.stages.iter().find(|s| s.id == id)
    }

    /// What the receiver's own stage is told to read, when it has been told
    /// anything. `None` leaves it on the head of the chain.
    pub fn tap(&self, builtin: u64) -> Option<Source> {
        self.feeding((builtin, 0))
    }

    /// Add a stage, unconnected. Wiring it up is a separate decision, since a
    /// stage dropped onto the canvas has no obvious input until one is drawn.
    pub fn add(&mut self, kind: &str) -> u64 {
        self.next += 1;
        let id = self.next;
        self.stages.push(Stage { id, kind: kind.to_string(), settings: Settings::new() });
        id
    }

    /// Remove a stage and every wire that touched it.
    ///
    /// Links into what it fed go too, rather than being rerouted round the
    /// gap: guessing that the stage after it wanted the stage before it is
    /// how an edit quietly builds something else.
    pub fn remove(&mut self, id: u64) {
        self.stages.retain(|s| s.id != id);
        self.links.retain(|l| l.to.0 != id && !matches!(l.from, Source::Stage(f, _) if f == id));
    }

    /// Feed an input port. An input takes one producer, so this replaces
    /// whatever was there, which is also what the graph builder does.
    pub fn connect(&mut self, from: Source, to: (u64, usize)) {
        if matches!(from, Source::Stage(f, _) if f == to.0) {
            return;
        }
        // A wire has to end somewhere that exists, and cannot start at one of
        // the receiver's own stages: the spectrum and the recorder consume
        // samples and produce nothing a patch could read.
        if !self.exists(to.0) {
            return;
        }
        if let Source::Stage(f, _) = from {
            if builtin::is(f) || !self.exists(f) {
                return;
            }
        }
        self.links.retain(|l| l.to != to);
        self.links.push(Link { from, to });
    }

    pub fn disconnect(&mut self, to: (u64, usize)) {
        self.links.retain(|l| l.to != to);
    }

    /// Take every wire off one output port.
    pub fn disconnect_from(&mut self, from: Source) {
        self.links.retain(|l| l.from != from);
    }

    pub fn feeding(&self, to: (u64, usize)) -> Option<Source> {
        self.links.iter().find(|l| l.to == to).map(|l| l.from)
    }

    /// A stage nothing reads is where a chain ends, which is where the packet
    /// bus has to be attached for anything it decodes to be seen.
    pub fn is_tail(&self, id: u64) -> bool {
        !self.links.iter().any(|l| matches!(l.from, Source::Stage(f, _) if f == id))
    }

    /// Whether a stage is still there to be wired to. The receiver's own
    /// stages always are; they are not the patch's to delete.
    fn exists(&self, id: u64) -> bool {
        builtin::is(id) || self.stage(id).is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deleting_a_stage_takes_its_wires_with_it() {
        // A wire to a stage that is gone is a graph that cannot be built, and
        // the failure would surface as the receiver refusing to rebuild
        // several actions later.
        let mut p = Patch::default();
        let a = p.add("mixer");
        let b = p.add("envelope");
        p.connect(Source::Span, (a, 0));
        p.connect(Source::Stage(a, 0), (b, 0));
        p.remove(a);
        assert!(p.links().is_empty(), "nothing may still refer to a deleted stage");
        assert_eq!(p.stages().len(), 1);
    }

    #[test]
    fn an_input_takes_one_producer() {
        let mut p = Patch::default();
        let a = p.add("mixer");
        let b = p.add("decimate");
        let c = p.add("envelope");
        p.connect(Source::Stage(a, 0), (c, 0));
        p.connect(Source::Stage(b, 0), (c, 0));
        assert_eq!(p.links().len(), 1, "the second wire replaces the first");
        assert_eq!(p.feeding((c, 0)), Some(Source::Stage(b, 0)));
    }

    #[test]
    fn a_stage_cannot_feed_itself() {
        // The graph builder would refuse the cycle at build time and take the
        // whole receiver down with it, which is a lot to pay for a slip of
        // the pointer.
        let mut p = Patch::default();
        let a = p.add("mixer");
        p.connect(Source::Stage(a, 0), (a, 0));
        assert!(p.links().is_empty());
    }

    #[test]
    fn an_id_is_never_reused() {
        // Otherwise a new stage inherits the wires of the one that was just
        // deleted, in a view where both look identical.
        let mut p = Patch::default();
        let a = p.add("mixer");
        p.remove(a);
        assert_ne!(p.add("mixer"), a);
    }
}
