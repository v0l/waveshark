#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

ceilings=tools/deps.ceiling

count() {
    local target=$1 features=$2
    cargo tree --locked -q -p app -p wave1090 --target "$target" \
        --no-default-features --features "$features" \
        -e normal,build --prefix none --format '{p}' |
        sed 's/ (\*)$//' | grep -v ' (/' | sort -u | wc -l
}

qualified() {
    sed 's/[^,]*/app\/&/g' <<<"$1"
}

shipped() {
    awk -v t="target: $1" '$0 ~ t { found = 1 } found && /features:/ { print $NF; exit }' \
        .github/workflows/build.yml
}

case ${1:-check} in
check)
    failed=0
    while read -r target features ceiling; do
        if [[ $(shipped "$target") != "$features" ]]; then
            echo "$target: $ceilings counts $features, the release builds $(shipped "$target")"
            failed=1
            continue
        fi
        n=$(count "$target" "$(qualified "$features")")
        if ((n > ceiling)); then
            echo "$target: $n crates, over the ceiling of $ceiling"
            failed=1
        elif ((n < ceiling)); then
            echo "$target: $n crates, under the ceiling of $ceiling; lower it with tools/deps.sh write"
            failed=1
        else
            echo "$target: $n crates"
        fi
    done <"$ceilings"
    exit $failed
    ;;
write)
    tmp=$(mktemp)
    while read -r target _ _; do
        features=$(shipped "$target")
        echo "$target $features $(count "$target" "$(qualified "$features")")"
    done <"$ceilings" >"$tmp"
    mv "$tmp" "$ceilings"
    cat "$ceilings"
    ;;
*)
    echo "usage: tools/deps.sh [check|write]" >&2
    exit 2
    ;;
esac
