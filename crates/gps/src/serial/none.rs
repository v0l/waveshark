use common::time::Duration;

pub(crate) struct Port;

impl std::io::Read for Port {
    fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
        Err(std::io::ErrorKind::Unsupported.into())
    }
}

impl std::io::Write for Port {
    fn write(&mut self, _: &[u8]) -> std::io::Result<usize> {
        Err(std::io::ErrorKind::Unsupported.into())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

pub(crate) fn open_port(path: &str, _: u32, _: Duration) -> std::io::Result<Port> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        format!("{path}: no serial ports on this platform"),
    ))
}

pub(crate) fn candidates() -> Vec<String> {
    Vec::new()
}
