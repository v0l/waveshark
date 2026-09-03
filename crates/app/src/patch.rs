//! A graph the operator drew, rather than one the scanner table implied.
//!
//! The receiver draws a patch for itself from what it is doing, and the
//! operator's changes to it are kept as [`Edits`] and put back on top of
//! whatever it draws next. The receiver rebuilds several times a minute (a
//! retune, a parameter that changes a rate, a channel opening) and a
//! description that lived only in the built graph would be lost at the first
//! one; so would a whole drawing kept from an earlier tuning, which is what
//! manual mode used to swap in.
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

    pub fn stage_mut(&mut self, id: u64) -> Option<&mut Stage> {
        self.stages.iter_mut().find(|s| s.id == id)
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
    /// table: where the edits file sits, named after the drawing that used
    /// to be saved whole.
    pub fn path() -> Option<std::path::PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(std::path::PathBuf::from)
            .or_else(|| {
                std::env::var_os("HOME").map(|h| std::path::PathBuf::from(h).join(".config"))
            })?;
        Some(base.join("waveshark").join("patch"))
    }

}

/// What the operator changed about the graph the receiver draws for itself.
///
/// Kept apart from the derived graph rather than as a second whole graph.
/// The derived one follows the dial, the scanner table and the strip; the
/// operator's changes are applied on top of whatever it is now. Saving the
/// whole drawing froze everything in it, so taking the graph over meant
/// taking over a stale tuning, a stale zoom and stale front ends as well,
/// and manual mode behaved like a different receiver. Now manual mode is a
/// lock on editing and nothing else: these edits apply in either mode.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Edits {
    /// Stages the operator added, with their settings.
    pub stages: Vec<Stage>,
    /// Derived stages the operator deleted.
    pub removed: Vec<u64>,
    /// Wires the operator drew or moved. Applied last, so each replaces
    /// whatever the derived graph had on that input.
    pub links: Vec<Link>,
    /// Inputs the operator pulled the wire off.
    pub unlinked: Vec<(u64, usize)>,
    /// Settings the operator changed on derived stages.
    pub settings: Vec<(u64, String, pipeline::param::ParamValue)>,
}

impl Edits {
    pub fn is_empty(&self) -> bool {
        self.stages.is_empty()
            && self.removed.is_empty()
            && self.links.is_empty()
            && self.unlinked.is_empty()
            && self.settings.is_empty()
    }

    /// Whether a setting on a derived stage is the operator's to override.
    ///
    /// A listening channel's stages and the audio bus's levels are the
    /// strip's: what the operator sets on them by hand goes back into the
    /// strip rather than sitting here as an override the strip would fight.
    /// The level of a bus input the strip did not set, which is a chain the
    /// operator drew, is the exception, since the strip has no other place
    /// to keep it.
    fn own_settings(st: &Stage, name: &str, base: &Stage) -> bool {
        if st.settings.contains_key("channel") {
            return false;
        }
        if st.kind == "audio_bus" {
            return (name.starts_with("vol") || name.starts_with("mute"))
                && !base.settings.contains_key(name);
        }
        true
    }

    /// What was changed, read off a graph edited from `base`.
    pub fn diff(full: &Patch, base: &Patch) -> Self {
        let mut e = Edits::default();
        for st in &full.stages {
            if !Patch::is_derived(st.id) {
                e.stages.push(st.clone());
                continue;
            }
            let Some(was) = base.stage(st.id) else { continue };
            for (name, v) in &st.settings {
                if was.settings.get(name) != Some(v) && Self::own_settings(st, name, was) {
                    e.settings.push((st.id, name.clone(), v.clone()));
                }
            }
        }
        for st in &base.stages {
            if full.stage(st.id).is_none() {
                e.removed.push(st.id);
            }
        }
        for l in &full.links {
            if !base.links.contains(l) {
                e.links.push(*l);
            }
        }
        for l in &base.links {
            let still = full.links.iter().any(|f| f.to == l.to);
            if !still && full.stage(l.to.0).is_some() {
                e.unlinked.push(l.to);
            }
        }
        e
    }

    /// Put the changes onto a derived graph.
    ///
    /// A wire to a stage the graph no longer derives is dropped quietly
    /// rather than refused: a front end that left the span takes its wires
    /// with it, and they come back when it does.
    pub fn apply(&self, p: &mut Patch) {
        for id in &self.removed {
            p.remove(*id);
        }
        for st in &self.stages {
            p.stages.retain(|s| s.id != st.id);
            p.stages.push(st.clone());
            p.next = p.next.max(st.id);
        }
        for (id, name, v) in &self.settings {
            if let Some(st) = p.stage_mut(*id) {
                st.settings.insert(name.clone(), v.clone());
            }
        }
        for to in &self.unlinked {
            p.disconnect(*to);
        }
        for l in &self.links {
            p.connect(l.from, l.to);
        }
    }

    /// `$XDG_CONFIG_HOME/waveshark/edits`, beside the session.
    pub fn path() -> Option<std::path::PathBuf> {
        Patch::path().map(|p| p.with_file_name("edits"))
    }

    /// The same plain lines the patch is written in, for the same reason.
    pub fn render(&self, places: &Places) -> String {
        let mut s = String::from("# waveshark edits: what was changed about the graph\n");
        for st in &self.stages {
            s.push_str(&format!("\nstage {} {}\n", st.id, st.kind));
            for (name, value) in &st.settings {
                s.push_str(&format!("set {name} {}\n", render_value(value)));
            }
        }
        s.push('\n');
        for id in &self.removed {
            s.push_str(&format!("removed {id}\n"));
        }
        for (id, name, value) in &self.settings {
            s.push_str(&format!("override {id} {name} {}\n", render_value(value)));
        }
        for (id, port) in &self.unlinked {
            s.push_str(&format!("unlink {id}:{port}\n"));
        }
        for l in &self.links {
            s.push_str(&format!("link {} {}:{}\n", render_source(l.from), l.to.0, l.to.1));
        }
        s.push('\n');
        for (id, (x, y)) in places {
            s.push_str(&format!("at {id} {x:.0} {y:.0}\n"));
        }
        s
    }

    pub fn parse(text: &str) -> (Self, Places) {
        let mut e = Edits::default();
        let mut places = Places::new();
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let mut w = line.split_whitespace();
            match w.next() {
                Some("stage") => {
                    let (Some(id), Some(kind)) = (w.next().and_then(|v| v.parse().ok()), w.next())
                    else {
                        continue;
                    };
                    e.stages.push(Stage { id, kind: kind.to_string(), settings: Settings::new() });
                }
                Some("set") => {
                    let (Some(name), Some(kind), Some(st)) = (w.next(), w.next(), e.stages.last_mut())
                    else {
                        continue;
                    };
                    let rest: Vec<&str> = w.collect();
                    if let Some(v) = parse_value(kind, &rest.join(" ")) {
                        st.settings.insert(name.to_string(), v);
                    }
                }
                Some("removed") => {
                    if let Some(id) = w.next().and_then(|v| v.parse().ok()) {
                        e.removed.push(id);
                    }
                }
                Some("override") => {
                    let (Some(id), Some(name), Some(kind)) =
                        (w.next().and_then(|v| v.parse().ok()), w.next(), w.next())
                    else {
                        continue;
                    };
                    let rest: Vec<&str> = w.collect();
                    if let Some(v) = parse_value(kind, &rest.join(" ")) {
                        e.settings.push((id, name.to_string(), v));
                    }
                }
                Some("unlink") => {
                    if let Some(to) = w.next().and_then(parse_port) {
                        e.unlinked.push(to);
                    }
                }
                Some("link") => {
                    let (Some(from), Some(to)) = (w.next(), w.next()) else { continue };
                    let (Some(from), Some(to)) = (parse_source(from), parse_port(to)) else {
                        continue;
                    };
                    e.links.push(Link { from, to });
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
        (e, places)
    }

    pub fn save(&self, places: &Places) {
        let Some(path) = Self::path() else { return };
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let _ = std::fs::write(path, self.render(places));
    }

    pub fn load() -> Option<(Self, Places)> {
        let text = std::fs::read_to_string(Self::path()?).ok()?;
        Some(Self::parse(&text))
    }
}

fn render_source(s: Source) -> String {
    match s {
        Source::Span => "span:0".to_string(),
        Source::Stage(id, port) => format!("{id}:{port}"),
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
    fn edits_are_what_a_drawing_differs_from_the_derived_graph_by() {
        use pipeline::param::ParamValue;
        // The operator's changes are read off an edited copy against the
        // graph the receiver drew, put back onto the next one it draws, and
        // written out and read back the same.
        let mut base = Patch::default();
        base.add_derived(Patch::DERIVED_BASE + 1, "dc_block", Settings::new());
        base.add_derived(Patch::DERIVED_BASE + 3, "spectrum", Settings::new());
        base.connect(Source::Span, (Patch::DERIVED_BASE + 1, 0));
        base.connect(Source::Stage(Patch::DERIVED_BASE + 1, 0), (Patch::DERIVED_BASE + 3, 0));

        let mut full = base.clone();
        let dec = full.add("decimate");
        full.stage_mut(dec).unwrap().settings.insert("factor".into(), ParamValue::Int(4));
        full.connect(Source::Span, (dec, 0));
        full.connect(Source::Stage(dec, 0), (Patch::DERIVED_BASE + 3, 0));
        full.remove(Patch::DERIVED_BASE + 1);
        full.stage_mut(Patch::DERIVED_BASE + 3)
            .unwrap()
            .settings
            .insert("size".into(), ParamValue::Int(4096));

        let e = Edits::diff(&full, &base);
        assert_eq!(e.stages.len(), 1);
        assert_eq!(e.removed, vec![Patch::DERIVED_BASE + 1]);
        assert_eq!(e.links.len(), 2, "{:?}", e.links);
        assert!(e.unlinked.is_empty(), "{:?}", e.unlinked);
        assert_eq!(e.settings.len(), 1);

        let mut again = base.clone();
        e.apply(&mut again);
        assert_eq!(again, full);
        // And a stage added afterwards cannot take an id the edits already
        // used, which would hand it another stage's wires.
        assert!(again.add("envelope") > dec);

        let mut places = Places::new();
        places.insert(dec, (10.0, 20.0));
        let (back, places_back) = Edits::parse(&e.render(&places));
        assert_eq!(back, e);
        assert_eq!(places_back, places);
        // No edits is no edits, so a fresh receiver is not told anything.
        assert!(Edits::diff(&base, &base).is_empty());
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
