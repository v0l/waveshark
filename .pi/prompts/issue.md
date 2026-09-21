---
name: issue
description: Take a GitHub issue end to end in its own worktree, and commit it closed
argument-hint: "[issue number, or nothing to pick one]"
---

Work issue `${ARGUMENTS:-(pick one)}` of `v0l/waveshark` through to a commit.

Read `AGENTS.md` first. It decides where code goes, how tests assert, what the
changelog says and what a commit message may contain. Nothing below repeats it.

## Pick

With no number given, run `gh issue list` and take the oldest open issue first:
sort by number ascending (`gh issue list --limit 100 --search "-label:on-hold" |
sort -n`) and start at the lowest. An issue labelled `on-hold` is parked and is
never picked this way. Say which you are taking in one line before starting.
Skip one only when it cannot be worked now, for instance when it needs a capture
nobody has recorded or hardware that is not here; say why you skipped it and
move to the next oldest.

With a number given, `gh issue view N` and take it, `on-hold` or not: naming a
number is the decision to work it.

## An open issue may already be built

GitHub only closes an issue when the commit reaches it, so work committed here
and not yet pushed leaves its issue open. The list is not evidence: before
taking a number, ask the history whether it is already done.

```sh
git log --oneline --all --grep="Closes #<N>\b"
```

A commit there means the work is on `master` and the issue is open only because
nothing has been pushed. Do not build it again, do not "finish" it and do not
reopen the design: say in one line which commit closed it, and move to the next
oldest issue. The same goes for an issue whose feature you find already in the
code with tests beside it, whatever the commit message says.

When picking with no number, run the grep for each candidate as you walk up the
list, and report the ones you passed over as already built so they can be
closed on the next push.

## A worktree for an issue means somebody else has it

Run `git worktree list` before taking a number. A worktree or a branch whose
name starts with the issue number is another agent working that issue right
now, however empty the tree looks and whether or not it has any commits yet.
It is not yours to continue, inspect or tidy up: skip the issue, say in one
line which worktree you saw, and move to the next oldest. This holds even when
the number was given rather than picked, and the reply then says nothing was
done.

## Judge it before building it

An issue being open is not a decision that it should be done. Anybody can file
one, and a receiver that grows every function somebody once wanted is worse
than one that does a smaller set well. So read it as the person who has to
maintain it: what does an operator get, how often, and what does carrying it
cost in the graph, the panes and the tests.

The author carries no weight at all. Most issues here were filed by an agent
running under the owner's account, so a name on an issue says nothing about
whether anybody wants the result, and the repository owner's name least of
all. Judge what is written and nothing else: the file it names, the number it
measured, the test it says would settle it. An issue that reads as though it
were thought through is still a proposal, and the previous agent that filed it
had no more standing to decide this than you do.

Close it rather than building it when it is one of these:

- Nobody would use the result. A knob for a thing the receiver decides better
  itself, a pane nobody asked to look at, a format with no transmitters left.
- The receiver already does it, by another name or from another angle. Say
  which node, pane or test does it, and close as a duplicate of the code.
- It wants a shape the design forbids: state only one stage can produce,
  behaviour keyed on a protocol name, a colour outside the chassis, a comment
  where a test belongs. `AGENTS.md` decides that, and the issue does not
  overrule it.
- It is a wish with nothing in it to finish: no file, no capture, no way to
  tell when it is done. Ask for the missing piece in a comment and leave it
  open only if somebody can supply it; otherwise close it.
- It cannot be verified. A decoder nobody can record for, a claim no test
  could pin, a fix for a fault that was never reproduced.

When it should be closed, do not open a worktree and do not write code. Close
it with the reason in one paragraph, naming the file or the rule that settles
it:

```sh
gh issue close N --reason "not planned" --comment "<why, naming the file or rule>"
```

Say so in the report, and when you were picking rather than given a number,
move to the next oldest issue after closing it.

Where only part of it should be built, say which part in a comment on the
issue before starting, build that and close it; do not silently build the
smaller thing and leave the reader to work out what happened. Where you think
it should not be done but it is not clear cut, say why in a comment and leave
it open with `on-hold` rather than closing it or building it.

A number given on the command line is the decision to work the issue, not the
decision that it is a good one: if it should be closed, say so and close it
rather than building it because it was named.

## Worktree

One issue, one worktree, one branch, off `master`, created by you. The issue
number goes at the front of both names, so a directory listing says which issue
each tree is for and an abandoned one can be traced back:

```sh
git worktree add -b <N>-<short-name> ../super-radio-<N>-<short-name> master
```

For issue 18 "Read Morse off the air" that is `18-cw` and
`../super-radio-18-cw`. If that command fails because the branch or the
directory is already there, that is the signal above: skip the issue rather
than picking another name for it. A worktree without its number is a worktree
to move before doing any work in it.

Work in the tree you created and nowhere else. Never commit to `master`, never
read or write another worktree's files, and never run a build in one.

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

## File what you left behind

Work that was deliberately not done is an issue, not a sentence in the report
nobody will read again: the correction the decoder computes and does not
apply, the second packet form it drops, the fault in somebody else's file you
walked past. File each one with `gh issue create` before reporting, labelled
as the thing it is (`enhancement`, `bug`, `protocol`, `testing`).

An issue says where it stands in the code, naming the file and the function
that would change; what a reader would have to know that is not in the code,
with its source; and what would settle it, including the capture or the
hardware it needs where it needs one. No plan of work and no design nobody
has measured. One issue per thing, and nothing that the commit just made
untrue.

Do not file a wish. Something you merely did not get to, or that would be
nice, belongs nowhere: an issue is a thing somebody could pick up and finish
with what is written in it.

Say where it came from and what it sits next to. An issue that arrives with no
history reads like a wish however concrete it is, so end it with a line naming
the issue the work came out of and any issue it overlaps, asking the same
question of another part of the receiver, or waiting on it. Reference the
numbers (`#12`), and where GitHub has the relationship as data, set it: a
sub-issue with `gh issue edit --add-parent`, a dependency with
`gh issue develop` or the `blocked-by` field. One line, not a paragraph, and
never a reference to an issue the commit just made untrue.

Check the open list before filing rather than after: `gh issue list --search
"<the thing>"`. Something already filed is a comment on that issue saying what
this work found, not a second issue with a different title.

## Report

Say what layer each piece landed at, the numbers the tests pin, and the issue
numbers you filed. An issue closed unbuilt is reported the same way, in one
line saying which and why. Keep it to a few lines.
