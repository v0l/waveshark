#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

profile=${PROFILE:-quick}
out=target/web
wasm=target/wasm32-unknown-unknown/$profile/waveshark.wasm

export CFLAGS_wasm32_unknown_unknown="-matomics -mbulk-memory -mmutable-globals"
export RUSTFLAGS="--cfg=web_sys_unstable_apis -C target-feature=+atomics,+bulk-memory,+mutable-globals \
-C link-arg=--shared-memory -C link-arg=--max-memory=4294967296 -C link-arg=--import-memory \
-C link-arg=--export=__wasm_init_tls -C link-arg=--export=__tls_size \
-C link-arg=--export=__tls_align -C link-arg=--export=__tls_base"

cargo_web() {
    rustup run nightly cargo "$1" -p app --no-default-features --features rtlsdr,hackrf,airspy \
        --target wasm32-unknown-unknown --profile "$profile" -Z build-std=panic_abort,std "${@:2}"
}

if [[ ${1:-} == check ]]; then
    cargo_web check "${@:2}"
    exit
fi

deploy=
strip=()
if [[ ${1:-} == deploy ]]; then
    deploy=1
    strip=(--remove-name-section --remove-producers-section)
    shift
fi

cargo_web build "$@"
rm -rf "$out"
wasm-bindgen --target web --no-typescript "${strip[@]}" --out-dir "$out" "$wasm"
env -u RUSTFLAGS -u CFLAGS_wasm32_unknown_unknown \
    cargo run -q --release -p webspin -- "$out/waveshark_bg.wasm" "$out/waveshark_bg.wasm"
sed -i "s|import('../../..')|import('../../../waveshark.js')|" \
    "$out"/snippets/wasm-bindgen-rayon-*/src/workerHelpers.js
cp crates/app/web/index.html crates/app/web/_headers "$out/"

protocols=${PROTOCOLS_DIR:-testdata/protocols}
if [[ ! -d $protocols ]]; then
    protocols=target/protocols
    if [[ -d $protocols/.git ]]; then
        git -C "$protocols" pull --quiet --ff-only || true
    else
        git clone --quiet --depth 1 "${PROTOCOLS_REPO:-https://github.com/v0l/waveshark-protocols.git}" "$protocols"
    fi
fi
rm -rf "$out/protocols"
mkdir -p "$out/protocols"
(cd "$protocols" && find . -name '*.yaml' -not -path './.git/*' | sed 's|^\./||' | sort) > "$out/protocols/index.txt"
while read -r rel; do
    mkdir -p "$out/protocols/$(dirname "$rel")"
    cp "$protocols/$rel" "$out/protocols/$rel"
done < "$out/protocols/index.txt"
echo "$out"

if [[ -n $deploy ]]; then
    wrangler pages deploy "$out" --project-name "${PAGES_PROJECT:-waveshark-app}" --branch main
fi
