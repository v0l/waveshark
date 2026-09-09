//! Puts the mark in the Windows executable, which is the only place a
//! program's icon is read from the file rather than set by the program: the
//! running window's icon comes from `window_icon` in `main.rs`, but Explorer,
//! the shortcut and the download in the browser all read this resource.
fn main() {
    println!("cargo:rerun-if-changed=../../assets/logo/waveshark-icon.ico");
    #[cfg(windows)]
    {
        let mut res = winresource::WindowsResource::new();
        res.set_icon("../../assets/logo/waveshark-icon.ico");
        if let Err(e) = res.compile() {
            println!("cargo:warning=icon not embedded: {e}");
        }
    }
}
