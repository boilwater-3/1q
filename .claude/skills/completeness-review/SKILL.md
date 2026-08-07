---
name: completeness-review
description: Review change completeness against the plan — correctness via agents, quality via simplify plugin, test coverage. Outputs a structured report with quality-gated merge flow.
argument-hint: "[plan file path]"
allowed-tools:
  - Bash
  - Read
  - Grep
  - Agent
  - Skill
---

# Completeness Review

Review code changes against the plan file across three dimensions, using plugin skills where available and agent-based fallback otherwise. After the review, a **quality gate** controls whether to proceed to merge — plan gaps, correctness issues, or build/test failures all block the merge path and direct the user back to a fix loop.

## Execution Flow

### Step 0: Determine scope

Find the plan file and diff range:

1. **Plan file**: use user-provided path, or search:
   ```bash
   ls -t .claude/plans/*.md .zcode/plans/*.md ~/.claude/plans/*.md 2>/dev/null | head -1
   ```

2. **Diff range**: check for uncommitted changes:
   ```bash
   git diff --stat HEAD && git diff --stat --cached
   ```
   If uncommitted changes exist → review working tree.
   Otherwise → `git diff main...HEAD`.

### Step 1: Collect changes

```bash
git diff --stat <range>
```
Group changed files by module (Modules, Shared headers, Shared impl, Tests, Docs).

### Step 2: Plan cross-reference (if plan exists)

Read the plan file. Extract action items / checklists. Cross-reference each item against changed files:

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

### Step 3: Deep review (parallel)

Run three review lanes in parallel. **Prefer plugin skills** where applicable; fall back to sub-agents.

#### Lane 1: Correctness (Agent-based, Opus)

Launch an Agent with **`model: "opus"`** to review the diff for:
- Logic errors, boundary conditions (nullptr, empty containers, division-by-zero, out-of-range)
- Error-handling completeness, RAII / resource leaks
- Compliance with CLAUDE.md Engineering Conventions (const correctness, no exceptions, namespace consistency)
- Architecture-level correctness: missing switch cases, inconsistent parallel implementations across files, state-machine invariants

> `code-review` works on PRs only and is NOT usable for uncommitted changes. Use Agent-based review for working-tree correctness checks.
> Lane 1 uses Opus because correctness review requires deeper architecture-level reasoning that flash models tend to miss (e.g., duplicate IsEccmDirective implementations drifting out of sync across files).

#### Lane 2: Code quality (Plugin: simplify)

**Directly invoke the simplify plugin:**
```
Skill("simplify")
```
The simplify plugin reviews recently modified code for reuse, simplification, naming, redundancy, and maintainability — it applies fixes directly. Capture its output as the quality findings.

#### Lane 3: Test coverage (Agent-based)

Launch an Agent to check:
- Do `tests/` directories contain new/modified tests matching the changed modules?
- If the plan requires new tests, are corresponding test cases present?
- Do tests cover critical paths and boundary conditions?

### Step 4: Report

Merge all findings into a structured report:

```markdown
## Completeness Review Report

### Plan Coverage
| Plan Item | Status | Risk |
|-----------|--------|------|

### Correctness Issues
- [file:line] Description

### Code Quality (simplify)
- Issues found and fixes applied by simplify

### Test Coverage
- Existing tests covering changes
- Missing test coverage

### Gate Status
- Plan gate: PASSED / BLOCKED (X partial, Y missing)
- Correctness gate: PASSED / BLOCKED (N issues)
- Test coverage gate: PASSED / BLOCKED (M gaps)
```

### Step 5: Quality gate

Evaluate three gates. **ALL must PASS** before the merge path opens. If any gate blocks, do NOT proceed to Step 6 — instead output the fix-loop guidance.

#### Gate A: Plan completeness
- PASS: all plan items are ✅. No ⚠️ or ❌.
- BLOCKED: any ⚠️ or ❌ items remain.

#### Gate B: Correctness
- PASS: Lane 1 found zero correctness issues.
- BLOCKED: any correctness issue exists.

#### Gate C: Test coverage
- PASS: Lane 3 found no missing test coverage.
- BLOCKED: critical paths lack tests.

#### If BLOCKED → fix loop

Output the blocking items and direct the user to iterate:

```
🚫 Merge blocked. Issues found:

  Plan:      2 items ⚠️ partial, 1 item ❌ missing
  Correctness: [file:line] nullptr dereference risk
  Test:      No tests for the new AR replay path

Fix the issues above, then re-run /completeness-review.
```

**Stop here.** Do not ask about merge, do not proceed to build.

#### If all PASS → proceed to Step 6

```
✅ All gates passed. Proceeding to build verification...
```

### Step 6: Build & test verification

Run the build and tests to verify Done Means. Use the CLADE.md prescribed workflow:

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

#### If build and tests pass → proceed to Step 7

### Step 7: Main sync check

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

After a clean rebase, re-run the build from Step 6 to confirm nothing broke.

#### If main is up to date

Proceed to Step 8.

### Step 8: Merge & cleanup

All gates passed, build/tests green, main is current. Declare the phase complete and ask:

> ✅ All checks passed. Plan complete, correctness verified, build & tests green.
>
> Merge `feature/<name>` into `main` and delete the feature branch?

**If user approves:**

```bash
git checkout main
git merge --no-ff feature/<name> -m "merge: <brief summary>"
git branch -d feature/<name>
```

Use `--no-ff` to preserve the feature branch as a distinct history marker. If the branch was pushed to remote, also run:

```bash
git push origin main
git push origin --delete feature/<name>
```

**If user declines:** leave the branch in place for the next session.

This step prevents branch proliferation — every merged feature branch is cleaned up immediately.

## Fallback: No Plan

If no plan file is found:
- Skip Step 2 (plan cross-reference).
- Step 5 Gate A (plan completeness) is SKIPPED (no plan to check against).
- Gates B and C still apply.
- Run Steps 1, 3–8 (collect changes → correctness + simplify + test coverage → report → correctness gate → build → main sync → merge prompt).
