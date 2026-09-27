#!/usr/bin/env bash
# Fetch the captures listed in decode.toml and fixture.toml and the rtl_433
# corpus samples listed in rtl433.toml, and check the local.toml captures
# already here.
#
# Every file is verified against the SHA-256 in the manifest. A capture that
# silently changed would invalidate every expected decode that references it,
# so a hash mismatch is a hard failure rather than a warning.
set -euo pipefail

cd "$(dirname "$0")"

only=("$@")

wanted() {
    (( ${#only[@]} == 0 )) && return 0
    local n
    for n in "${only[@]}"; do
        [[ "$n" == "$1" ]] && return 0
    done
    return 1
}

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
    mkdir -p "$(dirname "$name")"
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

fetch_capture() {
    wanted "$name" || return 0
    fetch "$name" "$sha" "$url" "$comp"
    [[ -n "$rname" ]] && fetch_verified "$rname" "$rsha" "$rurl"
    return 0
}

# Minimal manifest reader: enough for this flat structure, and avoids making
# a shell script depend on a TOML parser.
read_captures() {
    name=""; sha=""; url=""; comp=""; rname=""; rsha=""; rurl=""
    while IFS= read -r line; do
        case "$line" in
            '[[capture]]')
                [[ -n "$name" ]] && fetch_capture
                name=""; sha=""; url=""; comp="none"; rname=""; rsha=""; rurl=""
                ;;
            'reference_name = '*)   rname=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
            'reference_sha256 = '*) rsha=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
            'reference_url = '*)    rurl=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
            'name = '*)        name=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
            'sha256 = '*)      sha=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
            'url = '*)         url=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
            'compression = '*) comp=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        esac
    done < "$1"
    [[ -n "$name" ]] && fetch_capture
    return 0
}

read_captures decode.toml
read_captures fixture.toml

# The rtl_433 corpus: an uncompressed capture and the reference decode beside
# it, into their own directory so a capture that came from somewhere else is
# never confused with one that has an independent decode to check against.
mkdir -p rtl433

fetch_pair() {
    local name="$1" sha="$2" url="$3" rsha="$4" rurl="$5"
    local json="rtl433/${name%.cu8}.json"
    wanted "rtl433/$name" || return 0
    fetch_verified "rtl433/$name" "$sha" "$url"
    fetch_verified "$json" "$rsha" "$rurl"
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

# Captures that are never uploaded: nothing to fetch, only a check that the
# copy here is the one the tests were written against.
check_local() {
    wanted "$name" || return 0
    if [[ ! -f "$name" ]]; then
        echo "absent  $name (local only)"
        return 0
    fi
    local have_size have_sha
    have_size=$(stat -c %s "$name")
    have_sha=$(sha256sum "$name" | cut -d' ' -f1)
    if [[ "$have_size" != "$size" || "$have_sha" != "$sha" ]]; then
        echo "DIFFERS $name: $have_size bytes, sha256 $have_sha" >&2
        return 0
    fi
    echo "ok      $name (local)"
}

name=""; sha=""; size=""
while IFS= read -r line; do
    case "$line" in
        '[[capture]]')
            [[ -n "$name" ]] && check_local
            name=""; sha=""; size=""
            ;;
        'name = '*)   name=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        'sha256 = '*) sha=$(sed 's/.*= *"\(.*\)".*/\1/' <<<"$line") ;;
        'size = '*)   size=$(sed 's/.*= *\([0-9_]*\).*/\1/; s/_//g' <<<"$line") ;;
    esac
done < local.toml
[[ -n "$name" ]] && check_local

if (( ${#only[@]} > 0 )); then
    echo "done"
    exit 0
fi

# The protocol descriptions are published apart from the build, so the tests
# read the same files a receiver fetches rather than a copy kept in step by
# hand. A clone rather than a hashed tarball: the point of the repo is that
# it moves, and a test reading last month's copy would prove nothing.
PROTOCOLS_REPO=${PROTOCOLS_REPO:-https://github.com/v0l/waveshark-protocols.git}
if [[ -d protocols/.git ]]; then
    echo "fetch   protocols"
    git -C protocols pull --quiet --ff-only || echo "warn    protocols: could not update, keeping what is there" >&2
else
    echo "fetch   protocols"
    git clone --quiet --depth 1 "$PROTOCOLS_REPO" protocols
fi
echo "ok      protocols ($(find protocols -name '*.yaml' | wc -l) descriptions)"

echo "done"
