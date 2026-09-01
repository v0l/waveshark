use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-env-changed=LIMESUITE_LIB_DIR");
    println!("cargo:rerun-if-env-changed=LIMESUITE_INCLUDE_DIR");

    let mut clang_args: Vec<String> = Vec::new();

    if let Ok(dir) = env::var("LIMESUITE_LIB_DIR") {
        println!("cargo:rustc-link-search=native={dir}");
        println!("cargo:rustc-link-lib=LimeSuite");
        if let Ok(inc) = env::var("LIMESUITE_INCLUDE_DIR") {
            clang_args.push(format!("-I{inc}"));
        }
    } else {
        match pkg_config::Config::new().probe("LimeSuite") {
            Ok(lib) => {
                for p in &lib.include_paths {
                    clang_args.push(format!("-I{}", p.display()));
                }
            }
            Err(e) => {
                panic!(
                    "LimeSuite not found: {e}\n\
                     \n\
                     Install the development package:\n  \
                       Debian/Ubuntu:  sudo apt install liblimesuite-dev\n  \
                       Fedora:         sudo dnf install LimeSuite-devel\n  \
                       Arch:           sudo pacman -S limesuite\n  \
                       macOS:          brew install limesuite\n\
                     \n\
                     Or point at a custom build with LIMESUITE_LIB_DIR and \
                     LIMESUITE_INCLUDE_DIR.\n\
                     \n\
                     To build waveshark without LimeSDR support, disable the \
                     `limesdr` feature."
                );
            }
        }
    }

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_args(&clang_args)
        // LimeSuite.h pulls in LMS7002M_parameters.h, which is several hundred
        // file-scope `static const struct` definitions. Those are not exported
        // symbols, so anything generated from them fails to link. Take the LMS
        // API surface and nothing else.
        .allowlist_function("LMS_.*")
        .allowlist_type("lms_.*")
        .derive_debug(true)
        .derive_copy(true)
        .layout_tests(false)
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("failed to generate LimeSuite bindings");

    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings.write_to_file(out.join("bindings.rs")).expect("failed to write bindings");
}
