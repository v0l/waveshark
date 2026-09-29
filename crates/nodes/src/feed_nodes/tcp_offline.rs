use super::FeedSpec;

pub struct Link(std::convert::Infallible);

impl std::io::Read for Link {
    fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
        match self.0 {}
    }
}

pub fn connect(_: &FeedSpec) -> std::io::Result<Link> {
    Err(std::io::Error::new(std::io::ErrorKind::Unsupported, "this build reads no network feeds"))
}
