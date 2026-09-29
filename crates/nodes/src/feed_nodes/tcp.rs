use super::FeedSpec;

pub fn connect(spec: &FeedSpec) -> std::io::Result<std::net::TcpStream> {
    use std::net::ToSocketAddrs;
    let addr = spec
        .address()
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| std::io::Error::other("no address"))?;
    let sock = std::net::TcpStream::connect_timeout(&addr, common::time::Duration::from_secs(5))?;
    // A feed goes quiet at night; a read timeout is how the thread notices it
    // has been asked to stop rather than blocking until a frame arrives.
    sock.set_read_timeout(Some(common::time::Duration::from_secs(1)))?;
    Ok(sock)
}

#[cfg(test)]
mod tests {
    use super::super::*;

    const LONG: [u8; 14] =
        [0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3, 0x71, 0xc3, 0x2c, 0xe0, 0x57, 0x60, 0x98];

    #[test]
    fn a_feed_delivers_what_a_socket_sends_it() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        common::thread::spawn(move || {
            use std::io::Write;
            let (mut s, _) = listener.accept().unwrap();
            for _ in 0..3 {
                let _ = s.write_all(&beast_message(&LONG, 200, 0).unwrap());
            }
            let _ = s.flush();
            std::thread::sleep(common::time::Duration::from_millis(300));
        });

        let mut node = FeedNode::new(FeedSpec::new("127.0.0.1", port, &BEAST));
        node.negotiate(&[]).unwrap();
        let mut got = Vec::new();
        let deadline = common::time::Instant::now() + common::time::Duration::from_secs(5);
        while got.len() < 3 && common::time::Instant::now() < deadline {
            let mut out = vec![Payload::Packets(Vec::new())];
            let mut events = Vec::new();
            let tags = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &[], &tags, &mut events, &mut new_tags);
            node.process(&[], &mut out, &mut ctx).unwrap();
            got.extend(out[0].as_packets().unwrap().iter().cloned());
            std::thread::sleep(common::time::Duration::from_millis(20));
        }
        assert_eq!(got.len(), 3, "three frames sent, {} arrived", got.len());
        assert_eq!(got[0].bytes(), &LONG);
        assert_eq!(got[0].center_hz(), 1_090_000_000);
        assert_eq!(node.frames(), 3);
    }
}
