use super::ASKED;
use std::sync::atomic::{AtomicI32, Ordering};

/// Write end of the self-pipe the handler pokes, or -1 before install.
static WAKE: AtomicI32 = AtomicI32::new(-1);

extern "C" fn on_signal(_sig: libc::c_int) {
    // Everything here has to be async-signal-safe: a relaxed store to a
    // lock-free atomic, and a write to a pipe. No allocation, no locks, and
    // nothing that touches egui.
    ASKED.store(true, Ordering::Relaxed);
    let fd = WAKE.load(Ordering::Relaxed);
    if fd >= 0 {
        unsafe {
            libc::write(fd, [1u8].as_ptr().cast(), 1);
        }
    }
}

/// Catch SIGINT, SIGTERM and SIGHUP for the rest of the process.
///
/// The context is needed because an idle window is not repainting, and a
/// request nothing looks at is a receiver that ignores Ctrl-C until the mouse
/// moves. The waker thread turns the signal into a repaint, and the next
/// frame does the closing.
pub fn install(ctx: egui::Context) {
    let mut fds = [0 as libc::c_int; 2];
    // SAFETY: pipe fills the pair it is given, and the fds outlive the
    // process; the read end is owned by the thread below, the write end by
    // the handler.
    if unsafe { libc::pipe(fds.as_mut_ptr()) } != 0 {
        return;
    }
    let (rd, wr) = (fds[0], fds[1]);
    WAKE.store(wr, Ordering::Relaxed);

    unsafe {
        let mut act: libc::sigaction = std::mem::zeroed();
        act.sa_sigaction = on_signal as *const () as usize;
        // SA_RESETHAND is what makes the second Ctrl-C fatal: the handler is
        // put back to the default before this one runs.
        act.sa_flags = libc::SA_RESETHAND | libc::SA_RESTART;
        libc::sigemptyset(&mut act.sa_mask);
        for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP] {
            libc::sigaction(sig, &act, std::ptr::null_mut());
        }
    }

    common::thread::spawn(move || {
        let mut byte = [0u8; 1];
        loop {
            // SAFETY: reading one byte into a local buffer from the pipe this
            // thread owns.
            let n = unsafe { libc::read(rd, byte.as_mut_ptr().cast(), 1) };
            if n <= 0 {
                return;
            }
            ctx.request_repaint();
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shutdown::asked;

    #[test]
    fn an_interrupt_asks_rather_than_kills() {
        // If the handler is not installed this test does not fail, it takes
        // the whole test binary down, which is exactly what an interrupt
        // does to a receiver without one.
        install(egui::Context::default());
        assert!(!asked());
        unsafe { libc::raise(libc::SIGINT) };
        assert!(asked(), "the interrupt did not reach the frame loop");
    }
}
