---
name: merge
description: Merge every finished worktree into master, one at a time, and remove what is done
argument-hint: "[branch name, or nothing for all of them]"
---

Bring `${ARGUMENTS:-every finished worktree}` into `master` and clean up after
it.

Read `AGENTS.md` first. It decides what a commit message may say, what the
changelog holds and how tests assert. Nothing below repeats it.

## Survey first

Say what is out there before touching anything:

```sh
git worktree list
git branch --format='%(refname:short)' | while read b; do
  printf '%s ahead %s behind %s\n' "$b" \
    "$(git rev-list --count master..$b)" "$(git rev-list --count $b..master)"
done
```

A branch 0 ahead is already in: it needs no merge, only removing. A branch with
commits carries work, and whether that work is finished is the next question.

## Finished means clean, committed and tested where it was written

A branch is not finished because its commit message sounds complete. Check each
one before it goes in, and say which check answered:

- the worktree is clean (`git -C <dir> status --short` prints nothing),
- it carries a `CHANGELOG.md` line if anybody running the receiver would
  notice what it does.

Every worktree runs its own tests before its work is committed, so do not run
them again branch by branch here. The test run comes once, after everything is
merged.

Anything unfinished stays where it is. Say in one line what stopped it, and
move to the next branch rather than finishing somebody else's work here.

## One branch at a time, then one test run

Rebase the branch in its own worktree, then fast-forward `master` in the main
one. Never merge into a dirty tree, and never resolve a conflict by taking one
side of a file you have not read.

```sh
git -C ../super-radio-<branch> rebase master
git merge --ff-only <branch>
```

Run one branch at a time, never two at once: a rebase started before the
previous fast-forward lands on the old master. Note the commit each branch
landed at, so a failure later can be put back to the branch that caused it.

Conflicts are nearly always `CHANGELOG.md`, where two branches added a line to
the same `[Unreleased]` block: keep both lines. Where the two are the same
feature seen from different angles, keep one line saying what the reader gets,
as `AGENTS.md` requires. A conflict anywhere else is a real one and is read
properly.

When every finished branch is in, run the tests once:

```sh
XDG_CONFIG_HOME=$(mktemp -d) cargo test --workspace --no-fail-fast
```

The empty config directory matters: the radio tests read
`~/.config/waveshark`, and a receiver running on this machine writes edits
there that break them.

The run is the gate. Rerun a failing test alone before blaming a merge, since
the timing tests lose races under a full parallel run. A failure that stays is
the merge's problem however well the branch tested alone: find the branch by
running that one test at the commits noted above, then fix it in a commit of
its own on `master`, naming what the two sides disagreed about, or take that
branch back out by resetting to the commit before it and rebasing and
fast-forwarding the branches after it again.

Nothing here is pushed unless you are asked to push.

## Then remove what is done

A merged worktree is 10 to 100 GB of build artefacts and a branch nobody will
commit to again. Once a branch is 0 ahead of `master` and its tree is clean:

```sh
git worktree remove ../super-radio-<branch>
git branch -d <branch>
git worktree prune
```

`git branch -d` refuses a branch that is not merged, which is the check: never
reach for `-D` to get past it. Re-run the survey afterwards and say how much
disc came back (`df -h /`).

Leave a worktree alone when it is dirty, when its branch is ahead, or when it
is not yours to remove.

## Report

Per branch: merged or not and why, and what conflicted. Then the result of the
one test run, the count of commits now ahead of `origin/master`, and the space freed. A few
lines, not a table of every file.
