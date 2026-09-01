use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-env-changed=RTLSDR_LIB_DIR");
    println!("cargo:rerun-if-env-changed=RTLSDR_INCLUDE_DIR");

    let mut clang_args: Vec<String> = Vec::new();

    // Explicit override wins, so cross-compiles and vendored builds work
    // without pkg-config knowing anything about the sysroot.
    if let Ok(dir) = env::var("RTLSDR_LIB_DIR") {
        println!("cargo:rustc-link-search=native={dir}");
        println!("cargo:rustc-link-lib=rtlsdr");
        if let Ok(inc) = env::var("RTLSDR_INCLUDE_DIR") {
            clang_args.push(format!("-I{inc}"));
        }
    } else {
        match pkg_config::Config::new().atleast_version("0.6").probe("librtlsdr") {
            Ok(lib) => {
                for p in &lib.include_paths {
                    clang_args.push(format!("-I{}", p.display()));
                }
            }
            Err(e) => {
                panic!(
                    "librtlsdr not found: {e}\n\
                     \n\
                     Install the development package:\n  \
                       Debian/Ubuntu:  sudo apt install librtlsdr-dev\n  \
                       Fedora:         sudo dnf install rtl-sdr-devel\n  \
                       Arch:           sudo pacman -S rtl-sdr\n  \
                       macOS:          brew install librtlsdr\n\
                     \n\
                     Or point at a custom build with RTLSDR_LIB_DIR and \
                     RTLSDR_INCLUDE_DIR.\n\
                     \n\
                     To build waveshark without RTL-SDR support, disable the \
                     `rtlsdr` feature."
                );
            }
        }
    }

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_args(&clang_args)
        // Only take librtlsdr's own surface. Without this the bindings drag in
        // every libc declaration the header transitively pulls, which bloats
        // compile time and collides with other -sys crates.
        .allowlist_function("rtlsdr_.*")
        .allowlist_type("rtlsdr_.*")
        .allowlist_var("RTLSDR_.*")
        .default_enum_style(bindgen::EnumVariation::NewType {
            is_bitfield: false,
            is_global: false,
        })
        .derive_debug(true)
        .derive_copy(true)
        .layout_tests(false)
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("failed to generate librtlsdr bindings");

    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings.write_to_file(out.join("bindings.rs")).expect("failed to write bindings");
}
