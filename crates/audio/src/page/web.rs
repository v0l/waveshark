use crate::AudioError;

pub fn run<T: Send + 'static>(
    work: impl FnOnce() -> Result<T, AudioError> + Send + 'static,
) -> Result<T, AudioError> {
    if common::page::here() {
        return work();
    }
    let (tx, rx) = std::sync::mpsc::channel();
    common::page::run(Box::new(move || {
        Box::pin(async move {
            let _ = tx.send(work());
        })
    }))
    .map_err(AudioError::Cpal)?;
    rx.recv().map_err(|_| AudioError::Cpal("the page did not open the sound output".into()))?
}
