use common::time::Duration;
use std::io::{Read, Write};

/// Open a serial port and put it in the shape NMEA arrives in: eight bits, no
/// parity, one stop bit, no flow control, and raw so that a line is a line.
///
/// The termios call is what makes this a serial port rather than a file. A
/// USB CDC device ignores the baud rate and works either way, which is why
/// leaving it out appears to work; a real UART on a header does not, and
/// comes back as line noise that fails every checksum.
///
/// `idle` ends a read that has seen nothing, so a port with nothing on it
/// does not hang the thread forever. A feed wants ten seconds of patience; a
/// probe wants a fifth of a second and several tries.
pub(crate) struct Port {
    file: std::fs::File,
    was: libc::termios,
}

impl Read for Port {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.file.read(buf)
    }
}

impl Write for Port {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.file.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.file.flush()
    }
}

impl Drop for Port {
    fn drop(&mut self) {
        use std::os::unix::io::AsRawFd;
        let fd = self.file.as_raw_fd();
        // SAFETY: `fd` is open until `file` drops after this, and `was` is the
        // termios `tcgetattr` filled in when the port was opened.
        unsafe {
            libc::tcflush(fd, libc::TCIOFLUSH);
            libc::tcsetattr(fd, libc::TCSANOW, &self.was);
        }
    }
}

pub(crate) fn open_port(path: &str, baud: u32, idle: Duration) -> std::io::Result<Port> {
    use std::os::unix::fs::OpenOptionsExt;
    use std::os::unix::io::AsRawFd;
    let speed = match baud {
        4_800 => libc::B4800,
        9_600 => libc::B9600,
        19_200 => libc::B19200,
        38_400 => libc::B38400,
        57_600 => libc::B57600,
        115_200 => libc::B115200,
        other => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("unsupported baud rate {other}"),
            ));
        }
    };
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
        .open(path)?;
    let fd = file.as_raw_fd();
    // SAFETY: `fd` is open for the lifetime of `file`, and `tty` and `was`
    // are valid termios the calls only ever fill in or read.
    unsafe {
        if libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) != 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::ResourceBusy,
                format!("{path} is in use"),
            ));
        }
        let mut was: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut was) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        let mut tty = was;
        libc::cfmakeraw(&mut tty);
        libc::cfsetispeed(&mut tty, speed);
        libc::cfsetospeed(&mut tty, speed);
        tty.c_cflag |= libc::CLOCAL | libc::CREAD;
        tty.c_cflag &= !libc::CRTSCTS;
        // VTIME is in tenths of a second, and a zero would mean no timeout at
        // all, so anything under a tenth becomes one.
        tty.c_cc[libc::VMIN] = 0;
        tty.c_cc[libc::VTIME] = (idle.as_millis() / 100).clamp(1, 255) as _;
        if libc::tcsetattr(fd, libc::TCSANOW, &tty) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        let port = Port { file, was };
        let flags = libc::fcntl(fd, libc::F_GETFL);
        if flags < 0 || libc::fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK) != 0 {
            return Err(std::io::Error::last_os_error());
        }
        Ok(port)
    }
}

pub(crate) fn candidates() -> Vec<String> {
    let keep = ["ttyACM", "ttyUSB", "cu.usbmodem", "cu.usbserial", "cu.wchusbserial"];
    let mut found: Vec<String> = std::fs::read_dir("/dev")
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().to_string();
            keep.iter().any(|p| name.starts_with(p)).then(|| format!("/dev/{name}"))
        })
        .collect();
    found.sort();
    found
}
