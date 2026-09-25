#!/bin/sh
set -e
cd "$(dirname "$0")/../crates/limesdr-sys"
bindgen wrapper.h \
    --dynamic-loading LimeSuiteLib \
    --allowlist-function 'LMS_.*' \
    --allowlist-type 'lms_.*' \
    --no-layout-tests \
    -o src/bindings.rs \
    -- $(pkg-config --cflags LimeSuite)
