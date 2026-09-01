#!/usr/bin/env bash
# Fetch recorded IQ fixtures listed in fixtures.toml, and the rtl_433 corpus
# samples listed in rtl433.toml.
#
# Every file is verified against the SHA-256 in the manifest. A capture that
# silently changed would invalidate every expected decode that references it,
# so a hash mismatch is a hard failure rather than a warning.
set -euo pipefail

cd "$(dirname "$0")"

fetch() {
    local name="$1" sha="$2" url="$3" comp="$4"

    if [[ -f "$name" ]]; then
        local have
        have=$(sha256sum "$name" | cut -d' ' -f1)
        # The manifest hash is of the compressed upload, so an existing
        # decompressed file is accepted as-is.
        echo "ok      $name (already present)"
        return
    fi

    echo "fetch   $name"
    local tmp="${name}.${comp}"
    curl -fsSL -o "$tmp" "$url"

    local got
    got=$(sha256sum "$tmp" | cut -d' ' -f1)
    if [[ "$got" != "$sha" ]]; then
        echo "FAIL    $name: sha256 mismatch" >&2
        echo "        expected $sha" >&2
        echo "        got      $got" >&2
        rm -f "$tmp"
        exit 1
    fi

    case "$comp" in
        xz)   xz -d "$tmp" ;;
        gz)   gunzip "$tmp" ;;
        none) mv "$tmp" "$name" ;;
        *)    echo "FAIL    unknown compression: $comp" >&2; exit 1 ;;
    esac
    echo "ok      $name"
}

# Minimal manifest reader: enough for this flat structure, and avoids making
# a shell script depend on a TOML parser.
name=""; sha=""; url=""; comp=""
while IFS= read -r line; do
    case "$line" in
        '[[capture]]')
            [[ -n "$name" ]] && fetch "$name" "$sha" "$url" "$comp"
            name=""; sha=""; url=""; comp="none"
            ;;
        name*=*)        name=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        sha256*=*)      sha=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        url*=*)         url=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        compression*=*) comp=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
    esac
done < fixtures.toml
[[ -n "$name" ]] && fetch "$name" "$sha" "$url" "$comp"

# The rtl_433 corpus: an uncompressed capture and the reference decode beside
# it, into their own directory so a capture that came from somewhere else is
# never confused with one that has an independent decode to check against.
mkdir -p rtl433

fetch_pair() {
    local name="$1" sha="$2" url="$3" rsha="$4" rurl="$5"
    local json="rtl433/${name%.cu8}.json"
    fetch_verified "rtl433/$name" "$sha" "$url"
    fetch_verified "$json" "$rsha" "$rurl"
}

fetch_verified() {
    local path="$1" sha="$2" url="$3"
    if [[ -f "$path" ]]; then
        echo "ok      $path (already present)"
        return
    fi
    echo "fetch   $path"
    curl -fsSL -o "$path.part" "$url"
    local got
    got=$(sha256sum "$path.part" | cut -d' ' -f1)
    if [[ "$got" != "$sha" ]]; then
        echo "FAIL    $path: sha256 mismatch" >&2
        echo "        expected $sha" >&2
        echo "        got      $got" >&2
        rm -f "$path.part"
        exit 1
    fi
    mv "$path.part" "$path"
    echo "ok      $path"
}

name=""; sha=""; url=""; rsha=""; rurl=""
while IFS= read -r line; do
    case "$line" in
        '[[sample]]')
            [[ -n "$name" ]] && fetch_pair "$name" "$sha" "$url" "$rsha" "$rurl"
            name=""; sha=""; url=""; rsha=""; rurl=""
            ;;
        reference_sha256*=*) rsha=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        reference_url*=*)    rurl=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        name*=*)             name=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        sha256*=*)           sha=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        url*=*)              url=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
    esac
done < rtl433.toml
[[ -n "$name" ]] && fetch_pair "$name" "$sha" "$url" "$rsha" "$rurl"

echo "done"
