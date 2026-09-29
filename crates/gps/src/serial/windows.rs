use common::time::Duration;
use std::io::{Read, Write};

pub(crate) struct Port {
    file: std::fs::File,
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

/// The same port on Windows: a DCB instead of a termios, and read timeouts
/// set on the handle rather than in the terminal's control characters.
///
/// The state is read back before it is changed so that flow control stays
/// whatever the driver was installed with; only the four things NMEA fixes
/// are written, and a driver that wanted RTS/CTS keeps it.
pub(crate) fn open_port(path: &str, baud: u32, idle: Duration) -> std::io::Result<Port> {
    use std::os::windows::io::AsRawHandle;
    use windows_sys::Win32::Devices::Communication::{
        COMMTIMEOUTS, DCB, GetCommState, SetCommState, SetCommTimeouts,
    };

    // COM10 and above cannot be opened by name: only the first nine have a
    // DOS device alias, and the rest need the device namespace prefix.
    let name = if path.starts_with(r"\\.\") { path.to_string() } else { format!(r"\\.\{path}") };
    let file = std::fs::OpenOptions::new().read(true).write(true).open(&name)?;
    let handle = file.as_raw_handle() as isize as _;
    // SAFETY: `handle` is open for the lifetime of `file`, and both structs
    // are valid for the calls to fill in or read.
    unsafe {
        let mut dcb: DCB = std::mem::zeroed();
        dcb.DCBlength = std::mem::size_of::<DCB>() as u32;
        if GetCommState(handle, &mut dcb) == 0 {
            return Err(std::io::Error::last_os_error());
        }
        dcb.BaudRate = baud;
        dcb.ByteSize = 8;
        dcb.Parity = 0; // NOPARITY
        dcb.StopBits = 0; // ONESTOPBIT
        if SetCommState(handle, &dcb) == 0 {
            return Err(std::io::Error::last_os_error());
        }
        // Silence ends the read, as VTIME does on unix, so a port with
        // nothing on it does not hang the thread forever.
        let timeouts = COMMTIMEOUTS {
            ReadIntervalTimeout: 0,
            ReadTotalTimeoutMultiplier: 0,
            ReadTotalTimeoutConstant: idle.as_millis().min(u32::MAX as u128) as u32,
            WriteTotalTimeoutMultiplier: 0,
            WriteTotalTimeoutConstant: 0,
        };
        if SetCommTimeouts(handle, &timeouts) == 0 {
            return Err(std::io::Error::last_os_error());
        }
    }
    Ok(Port { file })
}

pub(crate) fn candidates() -> Vec<String> {
    // Windows has no directory of ports, and an absent COM port fails to
    // open immediately, so the first thirty-two are simply tried.
    (1..=32).map(|n| format!("COM{n}")).collect()
}
