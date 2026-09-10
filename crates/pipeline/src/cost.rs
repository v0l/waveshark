//! What a node costs to run, measured on every call and kept as a short
//! history, so the chain view can say where a slow graph goes rather than
//! that it is slow.
//!
//! The 95th percentile is what is shown. A mean hides the block that took
//! ten times the others, and that block is the one that made the radio drop
//! samples; a maximum is one outlier forever. Ninety-five in a hundred
//! recent calls is the number that has to fit inside a block for the
//! receiver to keep up.

/// Calls remembered per node. At a few hundred blocks a second this is under
/// a second of history, which is as far back as a reading anyone acts on.
pub const HISTORY: usize = 128;

/// A node's recent cost.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Cost {
    /// 95th percentile of one call, in microseconds.
    pub p95_us: u32,
    /// Mean of one call, in microseconds.
    pub mean_us: f32,
    /// Seconds of stream one call covers, or zero where the node runs on
    /// events rather than samples. `p95_us` against a million times this is
    /// the share of real time the node takes.
    pub block_s: f32,
    pub calls: u64,
}

impl Cost {
    /// How much of real time the node takes at its 95th percentile: one is a
    /// node that alone fills the block, and past one the receiver cannot
    /// keep up on this node's account.
    pub fn load(&self) -> Option<f32> {
        (self.block_s > 0.0).then(|| self.p95_us as f32 / (self.block_s * 1e6))
    }
}

/// The last [`HISTORY`] call durations of one node.
#[derive(Clone, Debug)]
pub struct Ring {
    us: [u32; HISTORY],
    at: usize,
    calls: u64,
    /// Exponentially smoothed, since a block's length varies with what the
    /// radio handed over and the cost is judged against the typical one.
    block_s: f32,
}

impl Default for Ring {
    fn default() -> Self {
        Self { us: [0; HISTORY], at: 0, calls: 0, block_s: 0.0 }
    }
}

impl Ring {
    pub fn push(&mut self, us: u32, block_s: f64) {
        self.us[self.at] = us;
        self.at = (self.at + 1) % HISTORY;
        self.calls += 1;
        if block_s > 0.0 {
            let b = block_s as f32;
            self.block_s =
                if self.block_s > 0.0 { self.block_s + 0.1 * (b - self.block_s) } else { b };
        }
    }

    pub fn cost(&self) -> Cost {
        let n = (self.calls as usize).min(HISTORY);
        if n == 0 {
            return Cost::default();
        }
        let mut v: Vec<u32> = self.us[..n].to_vec();
        v.sort_unstable();
        let p95 = v[((n - 1) * 95) / 100];
        let mean = v.iter().map(|&x| x as f64).sum::<f64>() / n as f64;
        Cost { p95_us: p95, mean_us: mean as f32, block_s: self.block_s, calls: self.calls }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_p95_is_the_slow_block_and_not_the_mean() {
        let mut r = Ring::default();
        for i in 0..100 {
            r.push(if i % 10 == 0 { 900 } else { 100 }, 0.01);
        }
        let c = r.cost();
        assert_eq!(c.p95_us, 900);
        assert!((c.mean_us - 180.0).abs() < 1.0);
        assert!((c.load().unwrap() - 0.09).abs() < 0.001);
    }

    #[test]
    fn an_event_node_has_no_load() {
        let mut r = Ring::default();
        r.push(5, 0.0);
        assert_eq!(r.cost().load(), None);
    }

    #[test]
    fn history_is_bounded() {
        let mut r = Ring::default();
        for _ in 0..HISTORY {
            r.push(1000, 0.01);
        }
        for _ in 0..HISTORY {
            r.push(10, 0.01);
        }
        assert_eq!(r.cost().p95_us, 10);
    }
}
