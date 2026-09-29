pub fn run<T: Send + 'static>(work: impl FnOnce() -> T + Send + 'static) -> T {
    work()
}
