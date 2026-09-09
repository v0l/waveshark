use candle_core::{Result, Tensor};
use candle_nn::{Linear, Module};

/// Thin wrapper around `candle_nn::Linear` that exposes a `forward` method
/// matching the same signature used across encoder and decoder, making the
/// calling code uniform regardless of bias presence.
pub(crate) struct LinearW(Linear);

impl LinearW {
    pub fn new(weight: Tensor, bias: Option<Tensor>) -> Self {
        Self(Linear::new(weight, bias))
    }

    pub fn forward(&self, x: &Tensor) -> Result<Tensor> {
        self.0.forward(x)
    }
}
