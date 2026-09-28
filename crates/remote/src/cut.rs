use common::{C32, SampleFormat};
use dsp::ddc::Ddc;

pub struct Cut {
    ddc: Option<Ddc>,
    wide: Vec<C32>,
}

impl Cut {
    pub fn new() -> Self {
        Self { ddc: None, wide: Vec::new() }
    }

    pub fn reach(span_hz: f64, rate: f64) -> f64 {
        ((span_hz - rate) / 2.0).max(0.0)
    }

    pub fn run(
        &mut self,
        uc8: &[u8],
        span: (f64, f64),
        want: (f64, f64),
        out: &mut Vec<C32>,
    ) -> f64 {
        let (span_center, span_rate) = span;
        let (want_center, want_rate) = want;
        let rate = want_rate.min(span_rate);
        let reach = Self::reach(span_rate, rate);
        let shift = (want_center - span_center).clamp(-reach, reach);
        self.wide.clear();
        SampleFormat::Cu8.convert(uc8, &mut self.wide);
        if rate >= span_rate && shift == 0.0 {
            self.ddc = None;
            out.extend_from_slice(&self.wide);
            return span_center;
        }
        let stale =
            self.ddc.as_ref().is_none_or(|d| d.rate_in() != span_rate || d.rate_out() != rate);
        if stale {
            self.ddc = Ddc::new(span_rate, rate, shift);
        }
        match &mut self.ddc {
            Some(d) => {
                if d.shift_hz() != shift {
                    d.set_shift(shift);
                }
                d.process(&self.wide, out);
                span_center + shift
            }
            None => {
                out.extend_from_slice(&self.wide);
                span_center
            }
        }
    }
}

impl Default for Cut {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cut_asked_past_the_edge_of_the_span_stops_at_the_edge() {
        let mut cut = Cut::new();
        let block = vec![128u8; 48_000];
        let mut out = Vec::new();
        let landed = cut.run(&block, (100e6, 2.4e6), (102e6, 240e3), &mut out);
        assert_eq!(landed, 100e6 + 1.08e6);
        assert_eq!(out.len(), 2_400);
    }

    #[test]
    fn the_whole_span_at_its_own_rate_is_handed_on_untouched() {
        let mut cut = Cut::new();
        let block: Vec<u8> = (0..=255).cycle().take(4_800).collect();
        let mut out = Vec::new();
        assert_eq!(cut.run(&block, (100e6, 2.4e6), (100e6, 2.4e6), &mut out), 100e6);
        let mut back = Vec::new();
        SampleFormat::Cu8.encode(&out, &mut back);
        assert_eq!(back, block);
    }
}
