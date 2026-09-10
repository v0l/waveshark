//! Span timing, so hot spots are measured rather than guessed.
//!
//! Accumulates wall time per span name and prints a table on demand. Spans are
//! coarse (one per pipeline stage or UI section) because the instrumentation
//! itself costs tens of nanoseconds and would dominate anything finer.

use std::collections::BTreeMap;
use std::sync::Mutex;
use std::time::{Duration, Instant};
use tracing::span::{Attributes, Id};
use tracing::Subscriber;
use tracing_subscriber::layer::Context;
use tracing_subscriber::registry::LookupSpan;
use tracing_subscriber::Layer;

#[derive(Default, Clone, Copy)]
pub struct Acc {
    pub total: Duration,
    pub calls: u64,
    pub max: Duration,
}

static ACC: Mutex<Option<BTreeMap<&'static str, Acc>>> = Mutex::new(None);

struct Entered(Option<Instant>);

pub struct Timing;

impl<S> Layer<S> for Timing
where
    S: Subscriber + for<'a> LookupSpan<'a>,
{
    fn on_new_span(&self, _a: &Attributes<'_>, id: &Id, ctx: Context<'_, S>) {
        if let Some(s) = ctx.span(id) {
            s.extensions_mut().insert(Entered(None));
        }
    }

    fn on_enter(&self, id: &Id, ctx: Context<'_, S>) {
        if let Some(s) = ctx.span(id) {
            if let Some(e) = s.extensions_mut().get_mut::<Entered>() {
                e.0 = Some(Instant::now());
            }
        }
    }

    fn on_exit(&self, id: &Id, ctx: Context<'_, S>) {
        let Some(s) = ctx.span(id) else { return };
        let started = s.extensions_mut().get_mut::<Entered>().and_then(|e| e.0.take());
        if let Some(t) = started {
            record(s.name(), t.elapsed());
        }
    }
}

pub fn record(name: &'static str, d: Duration) {
    if let Ok(mut g) = ACC.lock() {
        if let Some(map) = g.as_mut() {
            let e = map.entry(name).or_default();
            e.total += d;
            e.calls += 1;
            e.max = e.max.max(d);
        }
    }
}

/// Start collecting. Until this is called, `record` is a cheap no-op.
pub fn enable() {
    if let Ok(mut g) = ACC.lock() {
        *g = Some(BTreeMap::new());
    }
}

#[allow(dead_code)]
pub fn enabled() -> bool {
    ACC.lock().map(|g| g.is_some()).unwrap_or(false)
}

pub fn snapshot() -> Vec<(&'static str, Acc)> {
    let g = ACC.lock().ok();
    let mut v: Vec<_> = g
        .as_ref()
        .and_then(|g| g.as_ref())
        .map(|m| m.iter().map(|(k, v)| (*k, *v)).collect())
        .unwrap_or_default();
    v.sort_by(|a: &(&str, Acc), b| b.1.total.cmp(&a.1.total));
    v
}

/// Print a table of where the time went over `wall`.
pub fn report(wall: Duration) {
    let rows = snapshot();
    if rows.is_empty() {
        println!("no spans recorded");
        return;
    }
    // Wall time, so a span around a blocking call shows the time waited, not
    // CPU burned. rf_read sitting near 100% means the thread is idle in the
    // USB read, which is what it should be doing.
    println!(
        "\n{:<18} {:>10} {:>12} {:>11} {:>11} {:>10}",
        "span", "calls", "total ms", "us/call", "max us", "% of wall"
    );
    for (name, a) in rows {
        println!(
            "{:<18} {:>10} {:>12.1} {:>11.1} {:>11.1} {:>9.1}%",
            name,
            a.calls,
            a.total.as_secs_f64() * 1e3,
            a.total.as_secs_f64() * 1e6 / a.calls.max(1) as f64,
            a.max.as_secs_f64() * 1e6,
            a.total.as_secs_f64() / wall.as_secs_f64() * 100.0
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recording_is_inert_until_enabled() {
        // Not enabled in this test process by default, so nothing accumulates
        // and the instrumentation costs a lock and a branch.
        if !enabled() {
            record("never", Duration::from_secs(1));
            assert!(snapshot().is_empty());
        }
    }

    #[test]
    fn accumulation_sums_and_counts() {
        enable();
        record("alpha", Duration::from_millis(10));
        record("alpha", Duration::from_millis(30));
        record("beta", Duration::from_millis(5));
        let s = snapshot();
        let alpha = s.iter().find(|(n, _)| *n == "alpha").unwrap().1;
        assert_eq!(alpha.calls, 2);
        assert_eq!(alpha.total, Duration::from_millis(40));
        // Sorted by total time, so the heaviest span is first.
        assert_eq!(s[0].0, "alpha");
    }
}
