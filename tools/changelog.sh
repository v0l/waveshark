#!/usr/bin/env sh
# The changelog, as a release wants it.
#
#   tools/changelog.sh notes 0.2.0    print the [0.2.0] section, for release notes
#   tools/changelog.sh check 0.2.0    fail unless a [0.2.0] section exists
#   tools/changelog.sh release 0.2.0  turn [Unreleased] into [0.2.0] dated today
#                                     and open a new [Unreleased] above it
set -eu

log=$(dirname "$0")/../CHANGELOG.md
cmd=${1:?notes|check|release}
ver=${2:?version, without the v}
ver=${ver#v}

section() {
    # Lines from the version's heading up to the next heading or the link
    # references, heading excluded.
    awk -v v="$ver" '
        /^## \[/ { if (on) exit; on = ($0 ~ "^## \\[" v "\\]") ; next }
        /^\[/ && /\]: http/ { if (on) exit }
        on { print }
    ' "$log"
}

case "$cmd" in
notes)
    section | sed -e '1{/^$/d}' -e '${/^$/d}'
    ;;
check)
    if ! grep -q "^## \[$ver\]" "$log"; then
        echo "CHANGELOG.md has no [$ver] section" >&2
        exit 1
    fi
    if [ -z "$(section | tr -d '[:space:]')" ]; then
        echo "CHANGELOG.md's [$ver] section is empty" >&2
        exit 1
    fi
    ;;
release)
    if grep -q "^## \[$ver\]" "$log"; then
        echo "CHANGELOG.md already has [$ver]" >&2
        exit 1
    fi
    if [ -z "$(awk '/^## \[Unreleased\]/{on=1;next} /^## \[/{on=0} on' "$log" | tr -d '[:space:]')" ]; then
        echo "nothing under [Unreleased] to release" >&2
        exit 1
    fi
    today=$(date +%F)
    prev=$(grep -o '^## \[[0-9][^]]*\]' "$log" | head -1 | tr -d '#[] ')
    tmp=$(mktemp)
    awk -v v="$ver" -v d="$today" '
        /^## \[Unreleased\]/ { print; print ""; print "## [" v "] - " d; next }
        { print }
    ' "$log" > "$tmp"
    # The compare links: Unreleased now compares from the new tag, and the
    # new tag compares from the last one.
    sed -i.bak \
        -e "s|^\[Unreleased\]: .*|[Unreleased]: https://github.com/v0l/waveshark/compare/v$ver...HEAD\n[$ver]: https://github.com/v0l/waveshark/compare/v$prev...v$ver|" \
        "$tmp"
    rm -f "$tmp.bak"
    mv "$tmp" "$log"
    echo "CHANGELOG.md: [Unreleased] is now [$ver] - $today"
    ;;
*)
    echo "unknown command $cmd" >&2
    exit 2
    ;;
esac
