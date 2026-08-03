---
name: evidence-first-freeze-contract
description: Use when Codex is asked to plan, review, freeze, or implement high-risk algorithm or architecture changes where evidence should precede implementation. Trigger for SAR/ESR/EOS/radar/flight-dynamic modules, "冻结项", "证据矩阵", "先证明再实现", design gates, risky refactors, or cases where rejecting implementation is an acceptable outcome.
---

# Evidence-First Freeze Contract

Use this skill to prevent "implement first, justify later" behavior in high-risk technical work. The core rule is: Stage A builds an evidence matrix; Stage B implementation is allowed only if the evidence proves the need and scope. Evidence rejection is a valid final result.

This skill is the executable workflow for `docs/common/contract.md` section "证据优先开发模式". The contract defines the mandatory gate; this skill defines how to run it.

## Operating Rule

Do not start implementation during Stage A unless the user explicitly overrides the contract. If the user asks to "start implementing" a high-risk item without evidence, first run Stage A and report the gate result.

Use this workflow for:

- algorithms, physics models, signal/image processing, control logic, or maneuver logic;
- architecture refactors, module-internal responsibility changes, or dependency cleanup;
- output semantics, trace/replay/schema behavior, lifecycle/debug boundaries, or config semantics;
- any public `include/1q` API, module facade, builder, validator, or cross-domain contract change;
- test-threshold changes, skipped tests, known-limit splits, or acceptance-bar changes.

Do not use this workflow for:

- mechanical formatting;
- typo-only comments or documentation wording that does not change behavior;
- small local fixes where the failing behavior and acceptance condition are already explicit in the user request.

For every freeze item, answer four questions before editing production code:

1. What exact requirement, risk, or failure mode is being frozen?
2. What evidence would prove the requirement is real?
3. What evidence would disprove, narrow, or postpone it?
4. What is the smallest implementation boundary if the evidence passes?

## Workflow Summary

1. Stage A: prove, reject, narrow, or defer the need.
2. Contract Freeze: define the allowed implementation boundary and acceptance gates.
3. Stage B: implement only the frozen scope.
4. Stage C: verify, record actual results, and list residual freeze items.

## Stage A: Evidence Matrix

Create or update a matrix before proposing code changes. Prefer a repo planning file when one exists, such as `task_plan.md`, `findings.md`, `progress.md`, or a module-specific design note.

Use this structure:

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|-------------|------------|-----------------|------------|----------------|--------------------|----------|

Guidance:

- `Freeze item`: the decision under review, not the implementation idea.
- `Hypothesis`: the claim that must be true before implementation.
- `Evidence source`: code paths, tests, traces, docs, schemas, physics constraints, or user requirements.
- `Probe/Test`: concrete inspection, experiment, or focused test command.
- `Pass criterion`: objective condition that permits Stage B.
- `Rejection criterion`: objective condition that stops, narrows, or defers implementation.
- `Decision`: `pass`, `reject`, `narrow`, or `defer`.

Example:

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|-------------|------------|-----------------|------------|----------------|--------------------|----------|
| SAR focus-quality cleanup | Existing executor mixes quality evaluation with image formation and blocks isolated validation | `src/sar/session`, `src/sar/imaging`, focused image-quality tests | Inspect call graph and run focused tests | Quality computation has independent inputs/outputs and can be tested without changing `SarCycleResult` | Coupling is only naming/style and no behavior or testability risk is found | narrow |

## Evidence Quality

Prefer direct evidence in this order:

1. Existing failing behavior, trace divergence, reproducible test, or concrete acceptance requirement.
2. Source-level contract mismatch, schema/API inconsistency, or documented physical/system constraint.
3. Focused experiment or small characterization test.
4. Reasoned inference from code structure, explicitly labeled as inference.

Avoid treating broad intuition, aesthetic preference, or "future flexibility" as sufficient evidence for risky implementation.

Mark inference explicitly. For example: "Inference: this helper is a composition-root leak because the only caller is session assembly and it pulls pipeline internals into config validation."

## Stage A Outcomes

Stage A may end in any of these outcomes:

- `pass`: enter Stage B with a narrow implementation boundary.
- `reject`: do not implement; record why the requirement was not proven.
- `narrow`: implement only the proven subset.
- `defer`: identify missing evidence and the next probe needed.

When rejecting, be explicit and factual. A rejected implementation is not failure; it preserves engineering quality.

## Contract Freeze

Only create an implementation contract when Stage A has at least one `pass` or `narrow` decision. The contract must be written before production-code edits.

Use this template:

```markdown
## Frozen Contract

Proven requirement:
- ...

Allowed scope:
- Modules/directories:
- Classes/functions:
- Tests/docs:

Explicitly out of scope:
- Public headers:
- Cross-module types:
- Schema/trace/replay:
- Test thresholds/skips:
- Compatibility layers:

Behavior boundary:
- Inputs:
- Outputs:
- Errors/fallback:
- Lifecycle/debug/trace:

Acceptance gates:
- Build:
- Focused tests:
- Contract tests:
- Characterization tests:

Non-goals:
- ...
```

Rules:

- If implementation needs a file, module, public header, schema, or behavior not named in the contract, stop and return to Stage A.
- If a smaller implementation satisfies the proven requirement, choose the smaller implementation.
- Do not add speculative abstractions, compatibility layers, cross-module generalization, or public API surface unless the matrix proved they are needed.
- For this repo's current state, assume `include/1q` is mostly stable; prefer `src/<module>/` internal design cleanup unless evidence proves the public contract itself is wrong.

## Stage B: Implementation Gate

Before editing code, state:

- the proven requirement;
- the files or modules in scope;
- the files or modules explicitly out of scope;
- the validation commands that will prove the change.

Keep implementation aligned with the evidence.

Implementation rules:

- Fix real model, coordinate, state, config, data-flow, or boundary problems before touching tests.
- Do not weaken thresholds, broaden skips, or mark behavior unstable unless Stage A proved the original acceptance criterion was wrong.
- Add an abstraction only when it removes proven complexity, duplication, or an incorrect dependency boundary.
- Keep cross-domain symmetry meaningful: copy shape only when responsibilities match.
- Keep code, tests, and the relevant `docs/*/design.md` boundary text synchronized when behavior changes.
- Prefer focused tests first, then contract tests, then broader CI labels appropriate to the touched module.

## Stage C: Verification and Writeback

After Stage B, update the evidence matrix with actual results:

- changed files;
- tests or probes run;
- pass/fail result;
- residual risks;
- follow-up freeze items.

If tests fail, do not weaken thresholds or widen skips unless the evidence matrix proves the original acceptance criterion was wrong.

Use this verification record:

```markdown
## Stage C Result

Implemented scope:
- ...

Validation:
- `command`: pass/fail, important output

Residual risks:
- ...

Follow-up freeze items:
- ...
```

## Recommended Output Shape

For planning/review-only tasks:

1. Evidence matrix.
2. Decision per freeze item.
3. Frozen contract or Stage B scope only for passed/narrowed items.
4. Rejected/deferred items with next evidence needed.

For implementation tasks:

1. Brief Stage A result.
2. Frozen contract summary.
3. Stage B changes.
4. Verification commands and results.
5. Remaining freeze items.
