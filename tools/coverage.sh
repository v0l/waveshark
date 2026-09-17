#!/usr/bin/env bash
# Line coverage of the workspace, by crate and by file.
#
#   tools/coverage.sh                 run the tests instrumented, write the report
#   tools/coverage.sh -p dsp -p decode   only those crates
#   tools/coverage.sh gaps [N]        the N least-covered files (default 40)
#   tools/coverage.sh crates          coverage per crate
#   tools/coverage.sh open            open the HTML report in a browser
#   tools/coverage.sh clean           throw away the profile data
#
# gaps, crates and open read the last run's profile data, so they are cheap to
# repeat. Anything after -- goes to cargo test, e.g.
#
#   tools/coverage.sh -- --no-default-features
#
# The dev profile is used deliberately: tests that time the audio chain skip
# under debug_assertions, and an instrumented build cannot meet their
# deadlines. The signal path is still opt-level 3 in dev.
set -euo pipefail

cd "$(dirname "$0")/.."

out=target/llvm-cov
json=$out/coverage.json
html=$out/html/index.html

# Bindgen output and the test corpus loaders are not code anybody wrote.
ignore='(rtlsdr-sys|limesdr-sys|hackrf-usb)/.*bindings|/target/|tests/common'

fixture_notice() {
    local missing=0 total=0 name
    while read -r name; do
        total=$((total + 1))
        [[ -f "testdata/$name" ]] || missing=$((missing + 1))
    done < <(sed -n 's/^name = "\(.*\)"$/\1/p' testdata/fixtures.toml testdata/offair.toml)

    if ((missing > 0)); then
        echo
        echo "NOTE  $missing of $total recorded captures are absent, so the tests that"
        echo "      read them skipped and their decoders look colder than they are."
        echo "      ./testdata/fetch.sh fetches the corpus (about 1 GB)."
    fi
}

have_data() {
    [[ -f $json ]] || {
        echo "no coverage data yet: run tools/coverage.sh first" >&2
        exit 1
    }
}

# Per-file rows as "percent covered total path", from the llvm export JSON.
rows() {
    jq -r '.data[0].files[]
        | select(.summary.lines.count > 0)
        | [.summary.lines.percent, .summary.lines.covered, .summary.lines.count, .filename]
        | @tsv' "$json"
}

crate_table() {
    rows | awk -F'\t' '
        {
            path = $4
            sub(/.*\/crates\//, "", path)
            split(path, p, "/")
            covered[p[1]] += $2
            total[p[1]] += $3
        }
        END {
            for (c in total)
                printf "%6.1f%%  %6d / %-6d  %s\n",
                    100 * covered[c] / total[c], covered[c], total[c], c
        }
    ' | sort -n
}

case "${1:-run}" in
gaps)
    have_data
    n=${2:-40}
    echo "Least-covered files (uncovered lines first):"
    rows | awk -F'\t' '{
        printf "%6.1f%%  %5d uncovered  %s\n", $1, $3 - $2, $4
    }' | sort -k2 -rn | head -n "$n" |
        sed "s|$PWD/||"
    fixture_notice
    ;;
crates)
    have_data
    crate_table
    ;;
open)
    have_data
    [[ -f $html ]] || {
        echo "no HTML report: run tools/coverage.sh first" >&2
        exit 1
    }
    cargo llvm-cov report --open --html --output-dir "$out" --ignore-filename-regex "$ignore"
    ;;
clean)
    cargo llvm-cov clean --workspace
    rm -rf "$out"
    ;;
*)
    args=()
    packages=()
    while (($#)); do
        case "$1" in
        run) shift ;;
        -p | --package)
            packages+=(-p "$2")
            shift 2
            ;;
        --)
            shift
            args+=("$@")
            break
            ;;
        *)
            args+=("$1")
            shift
            ;;
        esac
    done
    scope=("${packages[@]:-}")
    ((${#packages[@]})) || scope=(--workspace)

    fixture_notice
    echo
    # One instrumented run, three reports off the same profile data. A test
    # that measures its own throughput can miss under instrumentation (the
    # DVB-T stage drains fewer transport packets), so a failure is carried to
    # the end rather than costing the whole report.
    status=0
    cargo llvm-cov test --no-report --no-fail-fast "${scope[@]}" "${args[@]}" || status=$?
    mkdir -p "$out"
    cargo llvm-cov report --json --output-path "$json" --ignore-filename-regex "$ignore"
    cargo llvm-cov report --lcov --output-path "$out/lcov.info" --ignore-filename-regex "$ignore"
    cargo llvm-cov report --html --output-dir "$out" --ignore-filename-regex "$ignore"

    echo
    crate_table
    echo
    echo "html   $html"
    echo "lcov   $out/lcov.info"
    echo "gaps   tools/coverage.sh gaps"
    fixture_notice
    if ((status != 0)); then
        echo
        echo "NOTE  the instrumented test run failed; coverage above is of what did run."
    fi
    exit "$status"
    ;;
esac
