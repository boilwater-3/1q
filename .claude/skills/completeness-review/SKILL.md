---
name: completeness-review
description: Review uncommitted code changes for correctness, code quality, and test coverage, then gate the merge. Triggered by the pre-commit hook for major C++ changes (≥3 files or ≥50 lines) and by "/completeness-review". Three parallel review lanes (correctness agent + bundled code-simplifier agent + test coverage agent) produce a structured report with a quality-gated merge flow.
argument-hint: "[optional diff range]"
allowed-tools:
  - Bash
  - Read
  - Grep
  - Agent
  - Skill
---

# Completeness Review

Review the current change set for correctness, code quality, and test coverage — against the plan
(when one exists) — with a quality-gated merge flow. The plan anchors the review: it provides the
change's intent, so agents can distinguish deliberate behavior changes from bugs, and can judge
whether the change is complete.

## Step 0: Determine scope

1. **Plan file**: use user-provided path, or search:
   ```bash
   ls -t .claude/plans/*.md .zcode/plans/*.md ~/.claude/plans/*.md 2>/dev/null | head -1
   ```
   If no plan file exists, ask the user for a one-sentence statement of the change's intent —
   it is the review anchor in place of a plan.

2. **Diff range**:
   ```bash
   git diff --stat HEAD && git diff --stat --cached
   ```
   If uncommitted changes exist → review working tree. Otherwise → `git diff main...HEAD`.

3. **Exclude unrelated user changes**: ask the user whether the working tree contains changes not
   belonging to this work; if so, restrict review to the stated files/modules. Do not fold unrelated
   changes into the review or the later merge flow.

## Step 1: Collect changes

```bash
git diff --stat <range>
```
Group changed files by module (Modules, Shared headers, Shared impl, Tests, Docs).

## Step 2: Plan cross-reference (if plan exists)

Read the plan. Extract action items / checklists. Cross-reference each item against changed files:

- ✅ **Done**: plan item has corresponding implementation with substance (not a stub)
- ⚠️ **Partial**: related files changed but missing core logic, stubbed out, no error handling
- ❌ **Missing**: plan item has no corresponding changes at all

Output a table:
```
| Plan Item | Status | Evidence File | Notes |
|-----------|--------|---------------|-------|
| Add AR replay codec | ✅ | src/airborne_radar/session/ar_replay_codec.cpp | Full implementation |
| Fix ESR memory leak | ⚠️ | src/electronic_surveillance_radar/processor.cpp | Comment only, no fix |
| Update SAR design.md | ❌ | - | No changes found |
```

Pass the plan's intent summary to all three review lanes as shared context.

## Step 3: Deep review (parallel)

Run three review lanes in parallel via the Agent tool. All lanes receive: the diff range, the
plan/intent summary (from Step 0/2), and the excluded-files list (from Step 0).

#### Lane 1: Correctness (Agent-based, Opus)

Launch an Agent with **`model: "opus"`** to review the diff for:
- Logic errors, boundary conditions (nullptr, empty containers, division-by-zero, out-of-range)
- Error-handling completeness, RAII / resource leaks
- Compliance with AGENTS.md Engineering Conventions (const correctness, no exceptions, namespace consistency)
- Architecture-level correctness: missing switch cases, inconsistent parallel implementations across files, state-machine invariants
- Divergence from the plan/intent: deliberate behavior changes should be checked against the plan,
  not flagged as bugs; missing plan pieces should be flagged as incompleteness

> Lane 1 uses Opus because correctness review requires deeper architecture-level reasoning that flash
> models tend to miss (e.g., duplicate implementations drifting out of sync across files).

#### Lane 2: Code quality (bundled agent: code-simplifier, Opus)

Read `agents/code-simplifier.md` and launch an Agent with **`model: "opus"`** and that prompt against
the recently modified code. It reviews for reuse, simplification, naming, redundancy, and
maintainability, applying project conventions (Google C++ Style, const-by-default, no exceptions,
preserved Chinese log annotations). **It reports findings only — it must not modify code during
review** (see agents/code-simplifier.md).

#### Lane 3: Test coverage (Agent-based)

Launch an Agent to check:
- Do `tests/` directories contain new/modified tests matching the changed modules?
- Do tests cover critical paths and boundary conditions?
- For new public API or significant logic changes: are new or updated tests present under `tests/`?

## Step 4: Verify and triage findings

Before writing the report, verify each finding from all lanes:

1. **Verify `[file:line]`**: confirm the cited location exists and the claim is accurate against
   live code. Drop findings that do not survive a direct check (misread, pre-existing, out of scope).
2. **Deduplicate**: merge overlapping findings across lanes (same issue reported by two lanes).
3. **Triage severity**:
   - **高 (blocking)** — breaks correctness, contract, data flow, or leaves the change incomplete
     (missing core plan piece, unhandled failure path, no tests for new public API)
   - **中 (recommended)** — real but non-blocking: cleanup, style drift, test gaps on non-critical paths
   - **低 (suggestion)** — nitpicks, naming, optional improvements

Only findings that survive verification enter the report. Report one severity per finding; if a
finding is disputed, mark it with the counter-evidence rather than silently dropping or keeping it.

## Step 5: Report

```markdown
## Completeness Review Report

### Plan Coverage
| Plan Item | Status | Risk |
|-----------|--------|------|

### Correctness Issues
- [severity] [file:line] Description

### Code Quality (code-simplifier)
- [severity] [file:line] Issues found (reported only, no code changes)

### Test Coverage
- Existing tests covering changes
- Missing test coverage

### Gate Status
- Plan gate: PASSED / BLOCKED
- Correctness gate: PASSED / BLOCKED (N blocking)
- Test coverage gate: PASSED / BLOCKED (M gaps)
- Recommended (non-blocking): N items — may be deferred to follow-up commits
```

## Step 6: Quality gate

Evaluate three gates. **Blocking criteria are severity-gated**: only 高 (blocking) findings block;
中/低 items are reported as recommendations and do not block the merge path.

#### Gate A: Plan completeness
- PASS: all plan items are ✅, or any ⚠️/❌ is explicitly accepted by the user (scope change).
- BLOCKED: a ❌ Missing or core ⚠️ Partial item exists and the user has not accepted it.

#### Gate B: Correctness
- PASS: zero 高 (blocking) correctness issues.
- BLOCKED: any 高 issue exists. 中/低 items do not block.

#### Gate C: Test coverage
- PASS: no missing tests on critical paths (new public API, significant logic changes).
- BLOCKED: critical paths lack tests. Gaps on non-critical paths are 中/低 recommendations.

#### If BLOCKED → fix loop

Output the blocking items and direct the user to iterate:

```
🚫 Merge blocked. Blocking issues:

  Plan:       [item] ❌ missing
  Correctness: [file:line] nullptr dereference risk (高)
  Test:      No tests for the new AR replay path (高)

Recommended (non-blocking): 3 items — see report.

Fix the blocking issues, then re-run /completeness-review.
```

**Stop here.** Do not ask about merge, do not proceed to build.

#### If all PASS → proceed to Step 7

```
✅ All gates passed. Proceeding to build verification...
```

## Step 7: Build & test verification

Verify Done Means with the repository-prescribed workflow. **Use the release preset** (AGENTS.md:
"Prefer release — JSBSim runs ~6× faster"):

```bash
preset="llvm-ninja-release-local"
log_prefix="/tmp/1q"

cmake --build --preset "$preset" >"${log_prefix}-review-build.log" 2>&1 || { tail -n 40 "${log_prefix}-review-build.log"; echo "BUILD FAILED"; }
ctest --preset "$preset" --output-on-failure -j 4 >"${log_prefix}-review-test.log" 2>&1 || { tail -n 40 "${log_prefix}-review-test.log"; echo "TEST FAILED"; }
```

Prefer focused testing when the change is module-scoped: run the touched modules' ctest labels first
(`-L unit -R <module>` or named targets), then the full suite. If a code-quality lane modified code,
**re-run Step 1–7 from the new working tree** instead of trusting the previous review.

#### If build fails or tests fail → fix loop

```
🚫 Build/test verification failed.

  Build: FAILED (see /tmp/1q-review-build.log)
  Tests: 3 failures (see /tmp/1q-review-test.log)

Fix the failures, then re-run /completeness-review.
```

**Stop here.** Do not ask about merge.

#### If build and tests pass → proceed to Step 8

## Step 8: Main sync check

Before merging, check whether `main` has moved since the branch was created:

```bash
git fetch origin main
git log --oneline feature/<name>..origin/main | head -5
```

#### If main has new commits

Rebase the feature branch onto the latest main:

```bash
git rebase origin/main
```

If rebase has conflicts, abort and report:

```
🚫 Rebase conflict. The following files have conflicts:
  - path/to/conflicted/file.cpp

Resolve the conflicts manually, then re-run /completeness-review.
```

After a clean rebase, re-run the build from Step 7 to confirm nothing broke.

#### If main is up to date

Proceed to Step 9.

## Step 9: Merge & cleanup

All gates passed, build/tests green, main is current. Before asking to merge, check the branch's
remote state:

```bash
git status -sb                      # ahead/behind vs upstream
git log --oneline @{u}..HEAD 2>/dev/null | head   # unpushed commits (fails if no upstream)
```

- If the branch has **no upstream** (never pushed) or has **unpushed commits**, tell the user the
  merge will delete local history beyond the merge record, and offer to push first:
  `git push -u origin feature/<name>`.
- Only delete the branch after the user confirms, and only if its commits are preserved (merged
  into main and/or pushed to remote).

Declare the phase complete and ask:

> ✅ All checks passed. Correctness verified, build & tests green.
>
> Merge `feature/<name>` into `main` and delete the feature branch?

**If user approves:**

```bash
git checkout main
git merge --no-ff feature/<name> -m "merge: <brief summary>"
git branch -d feature/<name>
```

Use `--no-ff` to preserve the feature branch as a distinct history marker. If the branch was pushed to
remote, also run:

```bash
git push origin main
git push origin --delete feature/<name>
```

**After merge, verify the merge result** (the merge itself can introduce resolution errors):

```bash
cmake --build --preset llvm-ninja-release-local >/tmp/1q-review-merge-build.log 2>&1 && \
  ctest --preset llvm-ninja-release-local --output-on-failure -j 4 >/tmp/1q-review-merge-test.log 2>&1
```

If the post-merge build/tests fail, stop and report instead of leaving main broken.

**If user declines:** leave the branch in place for the next session.

This step prevents branch proliferation — every merged feature branch is cleaned up immediately.
