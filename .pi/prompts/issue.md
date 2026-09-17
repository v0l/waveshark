---
name: issue
description: Take a GitHub issue end to end in its own worktree, and commit it closed
argument-hint: "[issue number, or nothing to pick one]"
---

Work issue `${ARGUMENTS:-(pick one)}` of `v0l/waveshark` through to a commit.

Read `AGENTS.md` first. It decides where code goes, how tests assert, what the
changelog says and what a commit message may contain. Nothing below repeats it.

## Pick

With no number given, run `gh issue list` and read the few that look bounded,
then say which you are taking and why in one line before starting. Prefer an
issue whose evidence can be synthesised or already sits in `testdata`, over one
that needs a capture nobody has recorded.

With a number given, `gh issue view N` and take it.

## Worktree

One issue, one worktree, one branch, off `master`:

```sh
git worktree add -b <short-name> ../super-radio-<short-name> master
```

Work there and nowhere else. Never commit to `master` and never touch another
worktree's files.

## Build it

Put each piece at the layer it belongs to: the waveform in `crates/dsp` named
after the modulation, the framing and the codes in their own modules, the
payload in `crates/decode`, the wiring in one `crates/nodes/src/*_nodes.rs`
with one `impl Protocol` and a line in `protocol::all()`. If a second protocol
using the same modulation could not call your new code without touching it, it
is in the wrong file.

Measure rather than assume. Where a threshold, a window length or a tolerance
had to be chosen, find the number by running it both ways and put the measured
figure in a comment beside the constant and in the test that pins it.

## Prove it

Tests assert counts and values, never `is_empty`. Pin how many frames decoded,
which callsigns and ids came out, and what a mistuned or noisy input does. A
new decoder gets a noise test: minutes of noise in, zero rows out.

Run, and report the numbers:

```sh
cargo test -p <crate> --lib
cargo fmt --all && cargo clippy -p <crate>
```

## Close it

One line under `[Unreleased]` in `CHANGELOG.md`, written for somebody deciding
whether to upgrade, under about fifteen words, naming the thing and not the
mechanism.

Then one commit:

```sh
git add -A
git commit -m "<imperative subject under 70 characters>" -m "Closes #N"
```

No trailers, no body beyond the `Closes` line unless there is a measured number
the diff cannot show. Do not push unless asked.

## Report

Say what layer each piece landed at, the numbers the tests pin, and anything
you found on the way that is worth its own issue. Keep it to a few lines.
