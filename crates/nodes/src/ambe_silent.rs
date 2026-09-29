pub(crate) struct Vocoder;

impl Vocoder {
    pub(crate) fn new() -> Self {
        Vocoder
    }
    pub(crate) fn reset(&mut self) {}
    pub(crate) fn decode_burst(
        &mut self,
        _frames: &[[u8; 9]; 3],
        _keystream: Option<&[bool; 49]>,
    ) -> Vec<f32> {
        Vec::new()
    }
    pub(crate) fn decode_channels(&mut self, _channels: &[[bool; 72]]) -> Vec<f32> {
        Vec::new()
    }
    pub(crate) fn decode_parameters(&mut self, _frames: &[[bool; 49]]) -> Vec<f32> {
        Vec::new()
    }
}
