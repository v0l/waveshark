//! Pass and block filters, as a chain draws them: FIR by design, IIR by
//! recursion.
//!
//! The FIR side is built out of [`crate::fir::lowpass`] rather than from its
//! own windowed sinc, because every response here is a lowpass with something
//! done to it: a highpass is a lowpass subtracted from an impulse, a bandpass
//! is the difference of two lowpasses, and a band stop is that subtracted from
//! an impulse in turn. Designing each separately would be three more chances
//! to get the window wrong.
//!
//! The IIR side is the RBJ cookbook biquad. It is here for the cases where a
//! few coefficients beat a few hundred taps: a de-emphasis, a rumble filter, a
//! notch on a carrier. It is not linear phase, which is why it is offered
//! beside the FIR rather than instead of it.

use crate::fir::lowpass;

/// What a filter keeps.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Response {
    /// Keep everything below the cutoff.
    Lowpass,
    /// Keep everything above it.
    Highpass,
    /// Keep a band around the centre.
    Bandpass,
    /// Keep everything except that band.
    Bandstop,
}

impl Response {
    pub fn from_name(s: &str) -> Option<Self> {
        Some(match s {
            "lowpass" | "low" => Self::Lowpass,
            "highpass" | "high" => Self::Highpass,
            "bandpass" | "band" | "pass" => Self::Bandpass,
            "bandstop" | "stop" | "notch" => Self::Bandstop,
            _ => return None,
        })
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Lowpass => "lowpass",
            Self::Highpass => "highpass",
            Self::Bandpass => "bandpass",
            Self::Bandstop => "bandstop",
        }
    }

    /// Whether the filter is described by a band rather than by one edge.
    pub fn is_band(self) -> bool {
        matches!(self, Self::Bandpass | Self::Bandstop)
    }
}

/// Taps for one filter, at a sample rate.
///
/// `centre_hz` is the cutoff for a lowpass or highpass and the middle of the
/// band otherwise; `width_hz` is the width of that band and is ignored by the
/// two edge responses. Frequencies are clamped inside the Nyquist rate, since
/// a cutoff outside it describes a filter that cannot exist and the caller
/// asking for one is usually a rate that changed under it.
pub fn design(
    response: Response,
    taps: usize,
    rate: f64,
    centre_hz: f64,
    width_hz: f64,
    atten_db: f64,
) -> Vec<f32> {
    let n = if taps.is_multiple_of(2) { taps + 1 } else { taps }.max(3);
    let nyquist = rate / 2.0;
    let clamp = |hz: f64| (hz.abs() / rate).clamp(1e-4, 0.4999);
    match response {
        Response::Lowpass => lowpass(n, clamp(centre_hz), atten_db),
        Response::Highpass => invert(lowpass(n, clamp(centre_hz), atten_db)),
        Response::Bandpass | Response::Bandstop => {
            let half = (width_hz.abs() / 2.0).min(nyquist * 0.98);
            let lo = clamp(centre_hz - half);
            let hi = clamp(centre_hz + half);
            let (lo, hi) = if hi > lo { (lo, hi) } else { (hi, lo + 1e-4) };
            let wide = lowpass(n, hi, atten_db);
            let narrow = lowpass(n, lo, atten_db);
            // The difference of two lowpasses is the band between them.
            let band: Vec<f32> = wide.iter().zip(&narrow).map(|(a, b)| a - b).collect();
            if response == Response::Bandpass {
                band
            } else {
                invert(band)
            }
        }
    }
}

/// Turn a filter into its complement: an impulse minus the filter, which
/// passes exactly what it stopped.
fn invert(mut h: Vec<f32>) -> Vec<f32> {
    let mid = h.len() / 2;
    for v in h.iter_mut() {
        *v = -*v;
    }
    h[mid] += 1.0;
    h
}

/// A second-order recursive section, RBJ cookbook.
///
/// Direct form I with its own state, so one of these filters one stream. A
/// complex stream runs two, one per component: the response is symmetric
/// about DC, which is what a filter described by a frequency rather than by a
/// pair of them means on an IQ stream.
#[derive(Clone, Copy, Debug, Default)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    /// `q` is the resonance: 0.707 is maximally flat, higher peaks at the
    /// cutoff, lower is gentler. For a band response it sets the width, and
    /// the caller usually has that in hertz instead: see [`Self::band_q`].
    pub fn design(response: Response, rate: f64, freq_hz: f64, q: f64) -> Self {
        let q = q.max(0.05);
        let w0 = std::f64::consts::TAU * (freq_hz.abs() / rate).clamp(1e-5, 0.4999);
        let (sin, cos) = (w0.sin(), w0.cos());
        let alpha = sin / (2.0 * q);
        let (b0, b1, b2, a0, a1, a2) = match response {
            Response::Lowpass => {
                let b1 = 1.0 - cos;
                (b1 / 2.0, b1, b1 / 2.0, 1.0 + alpha, -2.0 * cos, 1.0 - alpha)
            }
            Response::Highpass => {
                let b1 = -(1.0 + cos);
                ((1.0 + cos) / 2.0, b1, (1.0 + cos) / 2.0, 1.0 + alpha, -2.0 * cos, 1.0 - alpha)
            }
            Response::Bandpass => {
                (alpha, 0.0, -alpha, 1.0 + alpha, -2.0 * cos, 1.0 - alpha)
            }
            Response::Bandstop => (1.0, -2.0 * cos, 1.0, 1.0 + alpha, -2.0 * cos, 1.0 - alpha),
        };
        Self {
            b0: (b0 / a0) as f32,
            b1: (b1 / a0) as f32,
            b2: (b2 / a0) as f32,
            a1: (a1 / a0) as f32,
            a2: (a2 / a0) as f32,
            ..Default::default()
        }
    }

    /// The resonance that gives a band of `width_hz` around `freq_hz`, which
    /// is how a band filter is described everywhere except in the arithmetic.
    pub fn band_q(freq_hz: f64, width_hz: f64) -> f64 {
        if width_hz.abs() < 1e-9 {
            return 10.0;
        }
        (freq_hz.abs() / width_hz.abs()).clamp(0.05, 200.0)
    }

    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gain at one frequency, measured rather than derived: the point is that
    /// what the design does matches what the arithmetic above says it does.
    fn fir_gain(h: &[f32], rate: f64, hz: f64) -> f64 {
        let w = std::f64::consts::TAU * hz / rate;
        let (mut re, mut im) = (0.0, 0.0);
        for (i, c) in h.iter().enumerate() {
            re += *c as f64 * (w * i as f64).cos();
            im -= *c as f64 * (w * i as f64).sin();
        }
        (re * re + im * im).sqrt()
    }

    fn iir_gain(mut f: Biquad, rate: f64, hz: f64) -> f64 {
        // Long enough for the transient to leave, then peak over a whole
        // number of cycles.
        let n = 4096;
        let mut peak = 0.0f32;
        for i in 0..n {
            let t = i as f64 / rate;
            let y = f.process((std::f64::consts::TAU * hz * t).sin() as f32);
            if i > n / 2 {
                peak = peak.max(y.abs());
            }
        }
        peak as f64
    }

    #[test]
    fn a_lowpass_keeps_what_is_below_it_and_a_highpass_the_rest() {
        let rate = 48_000.0;
        let low = design(Response::Lowpass, 101, rate, 3_000.0, 0.0, 60.0);
        let high = design(Response::Highpass, 101, rate, 3_000.0, 0.0, 60.0);
        assert!(fir_gain(&low, rate, 500.0) > 0.9);
        assert!(fir_gain(&low, rate, 8_000.0) < 0.01);
        assert!(fir_gain(&high, rate, 500.0) < 0.01);
        assert!(fir_gain(&high, rate, 8_000.0) > 0.9);
    }

    #[test]
    fn a_band_filter_and_its_complement_are_opposites() {
        // The band stop is the band pass subtracted from an impulse, so
        // whatever one keeps the other has to lose. Getting this wrong gives
        // a filter that quietly passes everything.
        let rate = 48_000.0;
        let pass = design(Response::Bandpass, 201, rate, 5_000.0, 2_000.0, 60.0);
        let stop = design(Response::Bandstop, 201, rate, 5_000.0, 2_000.0, 60.0);
        assert!(fir_gain(&pass, rate, 5_000.0) > 0.9, "the band has to survive");
        assert!(fir_gain(&pass, rate, 500.0) < 0.02);
        assert!(fir_gain(&pass, rate, 15_000.0) < 0.02);
        assert!(fir_gain(&stop, rate, 5_000.0) < 0.02, "and be gone from the other");
        assert!(fir_gain(&stop, rate, 500.0) > 0.9);
        assert!(fir_gain(&stop, rate, 15_000.0) > 0.9);
    }

    #[test]
    fn a_biquad_notch_removes_the_carrier_it_is_pointed_at() {
        let rate = 48_000.0;
        let notch = Biquad::design(Response::Bandstop, rate, 1_000.0, Biquad::band_q(1_000.0, 100.0));
        assert!(iir_gain(notch, rate, 1_000.0) < 0.1, "the tone should go");
        assert!(iir_gain(notch, rate, 4_000.0) > 0.9, "and everything else stay");
    }

    #[test]
    fn a_biquad_lowpass_rolls_off_above_its_cutoff() {
        let rate = 48_000.0;
        let f = Biquad::design(Response::Lowpass, rate, 1_000.0, 0.707);
        assert!(iir_gain(f, rate, 200.0) > 0.9);
        // A single biquad is 12 dB an octave, so two octaves up is around a
        // twentieth rather than nothing at all.
        assert!(iir_gain(f, rate, 4_000.0) < 0.1);
    }
}
