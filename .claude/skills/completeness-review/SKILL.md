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

Review the current change set across three dimensions, using the bundled agents. After the review, a
**quality gate** controls whether to proceed to merge — correctness issues, or build/test failures
block the merge path and direct the user back to a fix loop.

## Execution Flow

### Step 1: Collect changes

```bash
git diff --stat HEAD && git diff --stat --cached
```
If uncommitted changes exist → review working tree.
Otherwise → `git diff main...HEAD`.

Group changed files by module (Modules, Shared headers, Shared impl, Tests, Docs).

### Step 2: Deep review (parallel)

Run three review lanes in parallel via the Agent tool.

#### Lane 1: Correctness (Agent-based, Opus)

Launch an Agent with **`model: "opus"`** to review the diff for:
- Logic errors, boundary conditions (nullptr, empty containers, division-by-zero, out-of-range)
- Error-handling completeness, RAII / resource leaks
- Compliance with AGENTS.md Engineering Conventions (const correctness, no exceptions, namespace consistency)
- Architecture-level correctness: missing switch cases, inconsistent parallel implementations across files, state-machine invariants

> Lane 1 uses Opus because correctness review requires deeper architecture-level reasoning that flash
> models tend to miss (e.g., duplicate implementations drifting out of sync across files).

#### Lane 2: Code quality (bundled agent: code-simplifier)

Read `agents/code-simplifier.md` and launch an Agent (general-purpose) with that prompt against the
recently modified code. It reviews for reuse, simplification, naming, redundancy, and maintainability,
applying project conventions (Google C++ Style, const-by-default, no exceptions, preserved Chinese log
annotations). Capture its output as the quality findings.

#### Lane 3: Test coverage (Agent-based)

Launch an Agent to check:
- Do `tests/` directories contain new/modified tests matching the changed modules?
- Do tests cover critical paths and boundary conditions?
- For new public API or significant logic changes: are new or updated tests present under `tests/`?

### Step 3: Report

Merge all findings into a structured report:

```markdown
## Completeness Review Report

### Correctness Issues
- [file:line] Description

### Code Quality (code-simplifier)
- Issues found and fixes applied

### Test Coverage
- Existing tests covering changes
- Missing test coverage

### Gate Status
- Correctness gate: PASSED / BLOCKED (N issues)
- Test coverage gate: PASSED / BLOCKED (M gaps)
```

### Step 4: Quality gate

Evaluate two gates. **ALL must PASS** before the merge path opens. If any gate blocks, do NOT proceed
to Step 5 — instead output the fix-loop guidance.

#### Gate A: Correctness
- PASS: Lane 1 found zero correctness issues.
- BLOCKED: any correctness issue exists.

#### Gate B: Test coverage
- PASS: Lane 3 found no missing test coverage.
- BLOCKED: critical paths lack tests.

#### If BLOCKED → fix loop

Output the blocking items and direct the user to iterate:

```
🚫 Merge blocked. Issues found:

  Correctness: [file:line] nullptr dereference risk
  Test:        No tests for the new AR replay path

Fix the issues above, then re-run /completeness-review.
```

**Stop here.** Do not ask about merge, do not proceed to build.

#### If all PASS → proceed to Step 5

```
✅ All gates passed. Proceeding to build verification...
```

### Step 5: Build & test verification

Run the build and tests to verify Done Means. Use the AGENTS.md prescribed workflow:

```bash
preset="llvm-ninja-debug-local"
log_prefix="/tmp/1q-review"

cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 40 "${log_prefix}-build.log"; echo "BUILD FAILED"; }
ctest --preset "$preset" --output-on-failure -j 4 >"${log_prefix}-test.log" 2>&1 || { tail -n 40 "${log_prefix}-test.log"; echo "TEST FAILED"; }
```

#### If build fails or tests fail → fix loop

```
🚫 Build/test verification failed.

  Build: FAILED (see /tmp/1q-review-build.log)
  Tests: 3 failures (see /tmp/1q-review-test.log)

Fix the failures, then re-run /completeness-review.
```

**Stop here.** Do not ask about merge.

#### If build and tests pass → proceed to Step 6

### Step 6: Main sync check

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

After a clean rebase, re-run the build from Step 5 to confirm nothing broke.

#### If main is up to date

Proceed to Step 7.

### Step 7: Merge & cleanup

All gates passed, build/tests green, main is current. Declare the phase complete and ask:

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

**If user declines:** leave the branch in place for the next session.

This step prevents branch proliferation — every merged feature branch is cleaned up immediately.
