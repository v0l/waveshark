use common::{CHANNEL_MATCH_HZ, ConversationKey, Error, Result};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::ParamValue;
use pipeline::port::{Payload, PortKind, StreamSpec};

pub const KIND: &str = "call_network";

const HOLD_S: f64 = 1.0;
const FORGET_S: f64 = 10.0;

struct Carried {
    key: ConversationKey,
    site_hz: f64,
    held: f64,
    sites: Vec<(f64, f64)>,
}

impl Carried {
    fn heard_on(&mut self, hz: f64, now: f64) {
        match self.sites.iter_mut().find(|(at, _)| (at - hz).abs() < CHANNEL_MATCH_HZ) {
            Some(site) => site.1 = now,
            None => self.sites.push((hz, now)),
        }
    }

    fn last(&self) -> f64 {
        self.sites.iter().map(|(_, t)| *t).fold(self.held, f64::max)
    }
}

pub struct NetworkNode {
    inputs: usize,
    clock: f64,
    carried: Vec<Carried>,
    dropped: u64,
}

impl Default for NetworkNode {
    fn default() -> Self {
        Self::new()
    }
}

impl NetworkNode {
    pub fn new() -> Self {
        Self { inputs: 1, clock: 0.0, carried: Vec::new(), dropped: 0 }
    }

    pub fn sites(&self, key: &ConversationKey) -> Vec<f64> {
        let mut out: Vec<f64> = self
            .carried
            .iter()
            .filter(|c| c.key.same_conversation(key))
            .flat_map(|c| c.sites.iter().map(|(hz, _)| *hz))
            .collect();
        out.sort_by(f64::total_cmp);
        out.dedup_by(|a, b| (*a - *b).abs() < CHANNEL_MATCH_HZ);
        out
    }

    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    pub fn tick(&mut self, seconds: f64) {
        self.clock += seconds;
        let now = self.clock;
        self.carried.retain(|c| now - c.last() < FORGET_S);
    }

    pub fn admits(&mut self, v: &common::Voice) -> bool {
        if v.network.is_none() || v.to.is_none() || v.pcm.is_empty() {
            return true;
        }
        let key = ConversationKey::of(v);
        let now = self.clock;
        let i = match self.carried.iter().position(|c| c.key.same_conversation(&key)) {
            Some(i) => i,
            None => {
                self.carried.push(Carried {
                    key: key.clone(),
                    site_hz: v.channel_hz,
                    held: now,
                    sites: Vec::new(),
                });
                self.carried.len() - 1
            }
        };
        let c = &mut self.carried[i];
        c.heard_on(v.channel_hz, now);
        if c.key.from.is_none() {
            c.key.from = key.from;
        }
        let here = (c.site_hz - v.channel_hz).abs() < CHANNEL_MATCH_HZ;
        if here || now - c.held >= HOLD_S {
            c.site_hz = v.channel_hz;
            c.held = now;
            true
        } else {
            self.dropped += 1;
            false
        }
    }
}

impl Node for NetworkNode {
    fn name(&self) -> &str {
        KIND
    }

    fn num_inputs(&self) -> usize {
        self.inputs.max(1)
    }

    fn optional_inputs(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        for (k, i) in inputs.iter().enumerate() {
            match i.spec.kind {
                PortKind::Voice => {}
                PortKind::Real if i.spec.is_silence() => {}
                other => {
                    return Err(Error::other(format!(
                        "the call network takes speech, and input {k} carries {other:?}"
                    )));
                }
            }
        }
        Ok(vec![StreamSpec {
            kind: PortKind::Voice,
            rate: super::OUT_HZ,
            center: common::Hz(0),
            bandwidth: 0.0,
            channels: 1,
            ..Default::default()
        }])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        self.tick(ctx.block_seconds);
        let Some(out) = outputs.first_mut() else { return Ok(()) };
        for p in inputs {
            let Payload::Voice(voices) = p else { continue };
            for v in voices {
                if self.admits(v) {
                    out.voice_mut().push(v.clone());
                }
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.clock = 0.0;
        self.carried.clear();
    }

    fn readings(&self) -> Vec<(String, String)> {
        let calls = self.carried.iter().filter(|c| c.sites.len() > 1).count();
        vec![
            ("on several sites".into(), calls.to_string()),
            ("copies dropped".into(), self.dropped().to_string()),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "inputs" => {
                let n = v.as_i64().ok_or_else(|| Error::other("expected a count"))?;
                self.inputs = n.max(1) as usize;
            }
            _ => return Err(Error::other(format!("call_network: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn site(hz: f64, to: &str, from: Option<&str>) -> common::Voice {
        common::Voice {
            system: "TETRA",
            channel_hz: hz,
            to: Some(to.into()),
            from: from.map(str::to_string),
            code: None,
            network: Some("272-91".into()),
            over: None,
            rate: 8_000.0,
            channels: 1,
            pcm: vec![0.25; 480],
        }
    }

    #[test]
    fn a_call_sent_by_four_sites_is_passed_on_from_the_first_until_it_goes_quiet() {
        let mut n = NetworkNode::new();
        let sites = [390.85e6, 391.925e6, 392.55e6, 393.3e6];
        let mut passed = Vec::new();
        for _ in 0..50 {
            n.tick(0.02);
            for hz in sites {
                if n.admits(&site(hz, "7309858", Some("7306696"))) {
                    passed.push(hz);
                }
            }
        }
        assert_eq!(passed.len(), 50, "one copy of each block");
        assert!(passed.iter().all(|hz| *hz == 390.85e6), "{passed:?}");
        assert_eq!(n.dropped(), 150);
        let key = ConversationKey::of(&site(391.925e6, "7309858", Some("7306696")));
        assert_eq!(n.sites(&key), sites);

        n.tick(HOLD_S - 0.02);
        assert!(!n.admits(&site(393.3e6, "7309858", Some("7306696"))), "held through a pause");
        n.tick(0.02);
        assert!(n.admits(&site(393.3e6, "7309858", Some("7306696"))), "a quiet site gives it up");
        assert!(!n.admits(&site(390.85e6, "7309858", Some("7306696"))));
    }

    #[test]
    fn traffic_that_does_not_name_its_caller_follows_the_site_that_did() {
        let mut n = NetworkNode::new();
        n.tick(0.02);
        assert!(n.admits(&site(390.85e6, "7309858", Some("7306696"))));
        assert!(!n.admits(&site(393.3e6, "7309858", None)));
        assert!(n.admits(&site(390.85e6, "7309858", None)));
    }

    #[test]
    fn two_callers_or_two_groups_are_two_calls_and_both_are_heard() {
        let mut n = NetworkNode::new();
        n.tick(0.02);
        assert!(n.admits(&site(390.85e6, "7309858", Some("7306696"))));
        assert!(n.admits(&site(393.3e6, "7309858", Some("7307867"))), "another caller");
        assert!(n.admits(&site(393.3e6, "7661987", Some("7661062"))), "another group");
        assert_eq!(n.dropped(), 0);
    }

    #[test]
    fn speech_that_names_no_network_is_never_held_back() {
        let mut n = NetworkNode::new();
        n.tick(0.02);
        let mut a = site(390.85e6, "7309858", Some("7306696"));
        a.network = None;
        let b = common::Voice { channel_hz: 393.3e6, ..a.clone() };
        assert!(n.admits(&a));
        assert!(n.admits(&b), "without a network two carriers are two places");
        let other = common::Voice {
            network: Some("234-14".into()),
            ..site(393.3e6, "7309858", Some("7306696"))
        };
        assert!(n.admits(&site(390.85e6, "7309858", Some("7306696"))));
        assert!(n.admits(&other), "another network's group of the same number");
    }
}
