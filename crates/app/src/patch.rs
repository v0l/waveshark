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
    /// Which stage the receiver considers the head of the chain: the samples
    /// after the DC block and the zoom, which is what everything derived from
    /// the dial is drawn against. A marker rather than a stage, because it
    /// names one rather than being one.
    pub const HEAD: u64 = u64::MAX;
    /// The span itself: the samples as the radio delivered them. Not a stage,
    /// but it is a box on screen and it is dragged and wired like one.
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

    /// Ids at or above this belong to the graph the receiver derives for
    /// itself. Derived ids are computed from what a stage is for, so that one
    /// keeps its node, its box and its wires across the rebuilds that derive
    /// it again; ids the operator's own stages take count up from one and
    /// cannot reach this far.
    pub const DERIVED_BASE: u64 = 1 << 40;

    pub fn is_derived(id: u64) -> bool {
        id >= Self::DERIVED_BASE && !builtin::is(id)
    }

    /// Add a stage, unconnected. Wiring it up is a separate decision, since a
    /// stage dropped onto the canvas has no obvious input until one is drawn.
    pub fn add(&mut self, kind: &str) -> u64 {
        self.next += 1;
        let id = self.next;
        self.stages.push(Stage { id, kind: kind.to_string(), settings: Settings::new() });
        id
    }

    /// Add a stage the receiver derives from what it is doing, under an id it
    /// chooses. Reusing the id across rebuilds is what lets the node itself,
    /// and the box it is drawn in, stay where they were.
    pub fn add_derived(&mut self, id: u64, kind: &str, settings: Settings) -> u64 {
        self.stages.retain(|s| s.id != id);
        self.stages.push(Stage { id, kind: kind.to_string(), settings });
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

/// Where a stage was put on screen, by the id it is drawn under.
///
/// Saved with the graph rather than beside it: an arrangement that came back
/// without its stages, or stages that came back in a heap, would both be
/// worse than starting from the automatic layout.
pub type Places = std::collections::BTreeMap<u64, (f32, f32)>;

impl Patch {
    /// `$XDG_CONFIG_HOME/waveshark/patch`, beside the session and the scanner
    /// table.
    pub fn path() -> Option<std::path::PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(std::path::PathBuf::from)
            .or_else(|| {
                std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".config"))
            })?;
        Some(base.join("waveshark").join("patch"))
    }

    /// Written as plain lines rather than through a serialisation crate, for
    /// the same reason the session is: a graph that will not load is exactly
    /// the situation where being able to read and fix the file matters.
    pub fn render(&self, places: &Places) -> String {
        let mut s = String::from("# waveshark patch: the graph as drawn\n");
        for st in &self.stages {
            s.push_str(&format!("\nstage {} {}\n", st.id, st.kind));
            for (name, value) in &st.settings {
                s.push_str(&format!("set {name} {}\n", render_value(value)));
            }
        }
        s.push('\n');
        for l in &self.links {
            let from = match l.from {
                Source::Span => "span:0".to_string(),
                Source::Stage(id, port) => format!("{id}:{port}"),
            };
            s.push_str(&format!("link {from} {}:{}\n", l.to.0, l.to.1));
        }
        s.push('\n');
        for (id, (x, y)) in places {
            s.push_str(&format!("at {id} {x:.0} {y:.0}\n"));
        }
        s
    }

    pub fn parse(text: &str) -> (Self, Places) {
        let mut p = Patch::default();
        let mut places = Places::new();
        let mut last: Option<u64> = None;
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut w = line.split_whitespace();
            match w.next() {
                Some("stage") => {
                    let id: u64 = match w.next().and_then(|v| v.parse().ok()) {
                        Some(v) => v,
                        None => continue,
                    };
                    let Some(kind) = w.next() else { continue };
                    p.stages.push(Stage {
                        id,
                        kind: kind.to_string(),
                        settings: Settings::new(),
                    });
                    // Ids handed out later must not land on one already in
                    // the file, or a new stage inherits its wires.
                    if !Self::is_derived(id) {
                        p.next = p.next.max(id);
                    }
                    last = Some(id);
                }
                Some("set") => {
                    let (Some(name), Some(kind), Some(id)) = (w.next(), w.next(), last) else {
                        continue;
                    };
                    let rest: Vec<&str> = w.collect();
                    if let Some(v) = parse_value(kind, &rest.join(" ")) {
                        if let Some(st) = p.stages.iter_mut().find(|s| s.id == id) {
                            st.settings.insert(name.to_string(), v);
                        }
                    }
                }
                Some("link") => {
                    let (Some(from), Some(to)) = (w.next(), w.next()) else { continue };
                    let (Some(from), Some(to)) = (parse_source(from), parse_port(to)) else {
                        continue;
                    };
                    p.links.push(Link { from, to });
                }
                Some("at") => {
                    let vals: Vec<&str> = w.collect();
                    if let [id, x, y] = vals[..] {
                        if let (Ok(id), Ok(x), Ok(y)) = (id.parse(), x.parse(), y.parse()) {
                            places.insert(id, (x, y));
                        }
                    }
                }
                _ => {}
            }
        }
        (p, places)
    }

    pub fn save(&self, places: &Places) {
        let Some(path) = Self::path() else { return };
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let _ = std::fs::write(path, self.render(places));
    }

    /// The graph last drawn, or nothing when none has been.
    pub fn load() -> Option<(Self, Places)> {
        let text = std::fs::read_to_string(Self::path()?).ok()?;
        let (p, places) = Self::parse(&text);
        (!p.stages.is_empty()).then_some((p, places))
    }
}

fn render_value(v: &pipeline::param::ParamValue) -> String {
    use pipeline::param::ParamValue as V;
    match v {
        V::Float(x) => format!("f {x}"),
        V::Int(x) => format!("i {x}"),
        V::Bool(x) => format!("b {x}"),
        V::Text(x) => format!("t {x}"),
        V::Choice(x) => format!("c {x}"),
    }
}

fn parse_value(kind: &str, rest: &str) -> Option<pipeline::param::ParamValue> {
    use pipeline::param::ParamValue as V;
    Some(match kind {
        "f" => V::Float(rest.parse().ok()?),
        "i" => V::Int(rest.parse().ok()?),
        "b" => V::Bool(rest == "true"),
        "t" => V::Text(rest.to_string()),
        "c" => V::Choice(rest.parse().ok()?),
        _ => return None,
    })
}

fn parse_source(s: &str) -> Option<Source> {
    if let Some(port) = s.strip_prefix("span:") {
        let _ = port;
        return Some(Source::Span);
    }
    let (id, port) = parse_port(s)?;
    Some(Source::Stage(id, port))
}

fn parse_port(s: &str) -> Option<(u64, usize)> {
    let (id, port) = s.split_once(':')?;
    Some((id.parse().ok()?, port.parse().ok()?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_drawn_graph_round_trips_through_its_file() {
        // A graph that came back missing a wire would be worse than one that
        // did not come back at all: the receiver would run, quietly, as
        // something else.
        let mut p = Patch::default();
        let mix = p.add("mixer");
        let env = p.add("envelope");
        p.stages_mut()[0]
            .settings
            .insert("shift_hz".into(), pipeline::ParamValue::Float(-125_000.0));
        p.connect(Source::Span, (mix, 0));
        p.connect(Source::Stage(mix, 0), (env, 0));
        let mut places = Places::new();
        places.insert(mix, (120.0, 340.0));
        let (back, back_places) = Patch::parse(&p.render(&places));
        assert_eq!(back, p);
        assert_eq!(back_places, places);
        // And a stage added afterwards cannot take an id the file already
        // used, which would hand it another stage's wires.
        let mut back = back;
        assert!(back.add("envelope") > env);
    }

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
