use common::{CHANNEL_MATCH_HZ, ConversationKey, Error, Result};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::ParamValue;
use pipeline::port::{Payload, PortKind, StreamSpec};

pub const KIND: &str = "call_network";

const HOLD_S: f64 = 0.2;
const FORGET_S: f64 = 10.0;
const PLAYED: usize = 40;

struct Carried {
    key: ConversationKey,
    site_hz: f64,
    held: f64,
    sites: Vec<(f64, f64)>,
    played: std::collections::VecDeque<u64>,
    waiting: Vec<(f64, common::Voice)>,
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

    pub fn pass(&mut self, v: &common::Voice) -> Vec<common::Voice> {
        if v.network.is_none() || v.to.is_none() || v.pcm.is_empty() {
            return vec![v.clone()];
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
                    played: Default::default(),
                    waiting: Vec::new(),
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
        c.waiting.retain(|(at, _)| now - at < 2.0 * HOLD_S);
        if !here && now - c.held < HOLD_S {
            c.waiting.push((now, v.clone()));
            self.dropped += 1;
            return Vec::new();
        }
        let mut out: Vec<common::Voice> = Vec::new();
        if !here {
            let since = c.held;
            let (mine, rest): (Vec<_>, Vec<_>) =
                std::mem::take(&mut c.waiting).into_iter().partition(|(at, w)| {
                    *at > since && (w.channel_hz - v.channel_hz).abs() < CHANNEL_MATCH_HZ
                });
            c.waiting = rest;
            self.dropped -= mine.len() as u64;
            out.extend(mine.into_iter().map(|(_, w)| w));
        }
        out.push(v.clone());
        c.site_hz = v.channel_hz;
        c.held = now;
        let mut fresh = Vec::new();
        for w in out {
            let frame = fingerprint(&w.pcm);
            if !here && c.played.contains(&frame) {
                self.dropped += 1;
                continue;
            }
            if c.played.len() == PLAYED {
                c.played.pop_front();
            }
            c.played.push_back(frame);
            fresh.push(w);
        }
        fresh
    }
}

fn fingerprint(pcm: &[f32]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for s in pcm {
        s.to_bits().hash(&mut h);
    }
    h.finish()
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
                let passed = self.pass(v);
                out.voice_mut().extend(passed);
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

    const FRAME_S: f64 = 0.06;

    fn site(hz: f64, to: &str, from: Option<&str>, frame: usize) -> common::Voice {
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
            pcm: (0..480).map(|i| ((frame * 480 + i) as f32 * 0.01).sin()).collect(),
        }
    }

    fn admits(n: &mut NetworkNode, v: &common::Voice) -> bool {
        !n.pass(v).is_empty()
    }

    fn call(hz: f64, frame: usize) -> common::Voice {
        site(hz, "7309858", Some("7306696"), frame)
    }

    #[test]
    fn a_call_sent_by_four_sites_is_passed_on_once_from_the_first() {
        let mut n = NetworkNode::new();
        let sites = [390.85e6, 391.925e6, 392.55e6, 393.3e6];
        let mut passed = Vec::new();
        for frame in 0..50 {
            n.tick(FRAME_S);
            for hz in sites {
                if admits(&mut n, &call(hz, frame)) {
                    passed.push(hz);
                }
            }
        }
        assert_eq!(passed.len(), 50, "one copy of each frame");
        assert!(passed.iter().all(|hz| *hz == 390.85e6), "{passed:?}");
        assert_eq!(n.dropped(), 150);
        let key = ConversationKey::of(&call(391.925e6, 0));
        assert_eq!(n.sites(&key), sites);
    }

    #[test]
    fn a_site_that_stops_sending_is_filled_in_from_one_that_did_not() {
        let mut n = NetworkNode::new();
        let (first, second) = (392.55e6, 390.85e6);
        let index = |v: &common::Voice| (0..30).find(|f| call(0.0, *f).pcm == v.pcm).unwrap();
        let mut heard = Vec::new();
        for frame in 0..30 {
            n.tick(FRAME_S);
            if !(10..20).contains(&frame) {
                heard.extend(n.pass(&call(first, frame)).iter().map(|v| (index(v), v.channel_hz)));
            }
            heard.extend(n.pass(&call(second, frame)).iter().map(|v| (index(v), v.channel_hz)));
        }
        let frames: Vec<usize> = heard.iter().map(|(f, _)| *f).collect();
        assert_eq!(frames, (0..30).collect::<Vec<_>>(), "every frame once, in order");
        assert!(heard[..10].iter().all(|(_, hz)| *hz == first));
        assert!(heard[10..].iter().all(|(_, hz)| *hz == second), "the fill keeps the call");
        assert_eq!(n.dropped(), 20, "each site's copies of what the other played");
    }

    #[test]
    fn a_frame_already_played_is_not_played_again_by_the_site_that_takes_over() {
        let mut n = NetworkNode::new();
        for frame in 0..5 {
            n.tick(FRAME_S);
            assert!(admits(&mut n, &call(390.85e6, frame)));
        }
        n.tick(HOLD_S);
        assert!(!admits(&mut n, &call(393.3e6, 4)), "a site running behind replays the last frame");
        assert!(admits(&mut n, &call(393.3e6, 5)));
        assert!(admits(&mut n, &call(393.3e6, 5)), "the site holding it is never second guessed");
    }

    #[test]
    fn traffic_that_does_not_name_its_caller_follows_the_site_that_did() {
        let mut n = NetworkNode::new();
        n.tick(FRAME_S);
        assert!(admits(&mut n, &call(390.85e6, 0)));
        assert!(!admits(&mut n, &site(393.3e6, "7309858", None, 1)));
        assert!(admits(&mut n, &site(390.85e6, "7309858", None, 1)));
    }

    #[test]
    fn two_callers_or_two_groups_are_two_calls_and_both_are_heard() {
        let mut n = NetworkNode::new();
        n.tick(FRAME_S);
        assert!(admits(&mut n, &call(390.85e6, 0)));
        assert!(admits(&mut n, &site(393.3e6, "7309858", Some("7307867"), 0)), "another caller");
        assert!(admits(&mut n, &site(393.3e6, "7661987", Some("7661062"), 0)), "another group");
        assert_eq!(n.dropped(), 0);
    }

    #[test]
    fn speech_that_names_no_network_is_never_held_back() {
        let mut n = NetworkNode::new();
        n.tick(FRAME_S);
        let a = common::Voice { network: None, ..call(390.85e6, 0) };
        let b = common::Voice { channel_hz: 393.3e6, ..a.clone() };
        assert!(admits(&mut n, &a));
        assert!(admits(&mut n, &b), "without a network two carriers are two places");
        let other = common::Voice { network: Some("234-14".into()), ..call(393.3e6, 0) };
        assert!(admits(&mut n, &call(390.85e6, 0)));
        assert!(admits(&mut n, &other), "another network's group of the same number");
    }
}
