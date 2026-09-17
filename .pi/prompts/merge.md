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

## Finished means it builds and its tests pass

A branch is not finished because its commit message sounds complete. Check each
one before it goes in, and say which check answered:

- the worktree is clean (`git -C <dir> status --short` prints nothing),
- it carries a `CHANGELOG.md` line if anybody running the receiver would
  notice what it does,
- it builds and its tests pass after being rebased onto master.

Anything unfinished stays where it is. Say in one line what stopped it, and
move to the next branch rather than finishing somebody else's work here.

## One branch at a time

Rebase the branch in its own worktree, then fast-forward `master` in the main
one. Never merge into a dirty tree, and never resolve a conflict by taking one
side of a file you have not read.

```sh
git -C ../super-radio-<branch> rebase master
git merge --ff-only <branch>
cargo test --workspace
```

Conflicts are nearly always `CHANGELOG.md`, where two branches added a line to
the same `[Unreleased]` block: keep both lines. Where the two are the same
feature seen from different angles, keep one line saying what the reader gets,
as `AGENTS.md` requires. A conflict anywhere else is a real one and is read
properly.

Then the tests, and they are the gate. A failure after a merge is the merge's
problem however well the branch tested alone: fix it in a commit of its own on
`master`, naming what the two sides disagreed about, or take the merge back out
with `git reset --hard` to the commit before it. Do not merge the next branch
over a failing tree.

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

Per branch: merged or not and why, what conflicted, and the test result. Then
the count of commits now ahead of `origin/master`, and the space freed. A few
lines, not a table of every file.
