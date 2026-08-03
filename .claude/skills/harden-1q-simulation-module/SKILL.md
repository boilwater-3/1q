---
name: harden-1q-simulation-module
description: Audit, repair, and close architecture or simulation-contract gaps in the 1Q C++ simulation modules (AR, ESR, SAR, EOS, SBIRS, and flight_dynamic). Use when docs may be stale, design and code disagree, runtime or next-cycle semantics change, cumulative state or snapshot ownership is risky, replay/schema/public DTOs must migrate together, truth-assisted or sensor-like behavior needs physical review, open questions require classification, or an approval-gated implementation must proceed from evidence through focused commit.
---

# Harden a 1Q Simulation Module

Turn uncertain review findings into an evidence-backed, approval-ready module contract. Preserve physical meaning, temporal semantics, deterministic replay, and clean ownership while avoiding speculative API growth.

## Load the right context

1. Read the repository `AGENTS.md` and honor explicit scope exclusions.
2. Read `docs/common/contract.md`, the owning module `design.md`, relevant `docs/common/open_questions.md` entries, and any `docs/review` draft.
3. Inspect live public headers, runtime code, state owners, schemas/codecs, tests, examples, and guards before trusting prose.
4. Read [references/repair-patterns.md](references/repair-patterns.md) when the task involves external decisions, next-cycle behavior, tracking modes, runtime migration, snapshot/replay, or simulation truth.
5. Read [references/verification-matrix.md](references/verification-matrix.md) before implementation or commit.

## Run the workflow

### 1. Freeze scope and authority

Restate:

- the module and owning `design.md`;
- review-only, plan-only, implementation, or commit authority;
- included and excluded subsystems;
- compatibility expectations;
- whether replay, public API, examples, and docs are in scope.

Do not infer permission to implement from a request to review or diagnose. If the user requests multiple subagents, split by independent module or surface and give each one a non-overlapping evidence task.

### 2. Build an evidence matrix

Create one row per claim or discrepancy:

| ID | Claim | Live code path | Test/replay evidence | Classification | Decision |
|---|---|---|---|---|---|
| X-01 | concise claim | producer → resolver → consumer | exact test/schema | current / stale / design-only / code drift | accept / reject / defer / open |

Trace every affected concept through these surfaces:

1. public config/input/result;
2. validation and runtime resolution;
3. cumulative state and ownership;
4. raw output versus attribution/debug/lifecycle;
5. snapshot, trace, schema, codec, comparator, replay;
6. tests, examples, batch validation, install/public guards, and authority docs.

Treat a declared field, enum, comment, or review statement as a hypothesis until the execution path consumes it.

Bound discovery before expanding scope:

- Locate the named fields/types with `rg` first.
- Inspect one decisive producer → resolver → consumer → replay/schema → focused-test path.
- If the named premise is absent from live code, classify it as unverified and stop; do not audit the whole module to invent an equivalent problem.
- For review-only or plan-only work, name remaining evidence gaps instead of recursively opening unrelated subsystems.

### 3. Separate stale facts from design disagreements

Classify before editing:

- **Class A — stale fact or stale evidence:** Correct the authority text against live behavior. Do not redesign merely to preserve old prose.
- **Class B — design/code disagreement:** Compare the current implementation and proposed design using architecture, simulation fidelity, state/replay cost, and testability. Freeze the owner's decision before implementation.
- **Class C — verified defect:** Define the smallest cross-surface repair and its behavioral proof.
- **Class D — useful but unproven capability:** Register an open question with evidence, current boundary, and re-entry gate.

Use reject and defer as valid outcomes. Do not turn every review observation into work.

### 4. Explain the decision before freezing it

For a non-expert owner, explain:

1. what exists now;
2. what the earlier design intended;
3. what would change;
4. which part is real sensor/physics behavior and which part is simulation scaffolding;
5. the failure mode prevented by the change.

Use one concrete analogy when timing, state ownership, or truth-assisted behavior is otherwise hard to follow.

### 5. Freeze the behavioral contract

Record these decisions explicitly before changing code:

- source of truth and provenance identifiers;
- same-cycle versus next-successful-cycle timing;
- replace, merge, accumulate, or disable semantics;
- behavior across rejected, standby, powered-off, and replay-only cycles;
- sole owner of every cumulative state and snapshot field;
- public DTO versus internal algorithm types;
- raw output versus attribution/debug/lifecycle fields;
- runtime patch preservation, reset, retag, release, and no-op rules;
- replay events and comparator obligations;
- compatibility or deliberate incompatibility;
- behavioral test oracle.

Prefer enums or tagged states over interacting booleans when states are mutually exclusive. Keep orthogonal choices separate, such as simulation mode versus estimator backend.

### 6. Implement in bounded slices

Follow the repository file-count limit and build between slices. Prefer this order when all surfaces change:

1. public types and validation;
2. schema, codec, comparator, and unknown-value rejection;
3. internal state, ownership, snapshot, and deterministic random streams;
4. runtime behavior and state migration;
5. attribution/debug/lifecycle, tests, examples, and authority docs;
6. remove transitional fields and run full residual searches.

Keep intermediate compatibility scaffolding only long enough to retain buildable checkpoints. Remove it from the final result when the project is explicitly pre-release and compatibility is not required.

For multi-component migration, validate a temporary candidate state before replacing live state. Do not invent session rollback where the execution path has no post-mutation failure.

### 7. Prove behavior, not plumbing

Require tests that observe the intended consequence:

- For control decisions, prove detection margin, signal behavior, or downstream control effect rather than only profile fields.
- For next-cycle behavior, cover accepted input, stale/mismatched provenance, empty replacement, rejected cycles, powered-off cycles, snapshot continuation, and replay.
- For sensor modes, prove physical gates and state transitions independently from reported measurement noise.
- For runtime patches, prove exactly which locks, filters, counters, actuators, cues, phases, and random streams survive or reset.
- For random behavior, use fixed-width state, stable domain tags, explicit consumption rules, snapshot continuation, and seed-local reset tests.

Do not weaken thresholds, skips, or assertions merely to obtain green tests.

### 8. Perform a post-implementation architecture and simulation audit

Check:

- Can invalid public combinations still be represented?
- Does simulation truth leak into a sensor-facing raw output or estimator decision?
- Can display noise influence pointing, detection gates, or state transitions unintentionally?
- Does a global random stream create undocumented target-order dependence?
- Does a mode name claim more physical realism than the model provides?
- Can replay reproduce pending decisions, mode changes, random consumption, and non-executed cycles?
- Does each state have one owner and one restoration path?

Fix verified blockers. Put non-blocking fidelity questions in `docs/common/open_questions.md` with a measurable Stage A re-entry condition.

### 9. Close documentation cleanly

- Put settled module behavior in the owning `design.md`.
- Put cross-module rules in `docs/common/contract.md`.
- Keep unresolved, non-normative questions in `docs/common/open_questions.md`.
- Keep `docs/review` files draft-only; delete a settled draft after migrating durable conclusions.
- Cite exact live tests as evidence. Do not leave a second authority document.

### 10. Validate and hand off

Discover target names from `tests/cmake/TestTargets.cmake` or live build metadata; never guess them. Build before running CTest. Use the repository-preferred release preset unless the task requires another preset.

Run:

- focused unit/integration/contract/replay targets;
- module CTest labels and batch validation;
- public API, install, C++11 header, dependency-isolation, and docs guards when affected;
- old-name/old-field residual searches;
- `git diff --check` and worktree scope inspection.

Report blockers separately from warning-only physical trends. Commit only when asked, with unrelated user changes excluded.

## Stop conditions

Stop and request a decision when:

- two plausible semantics produce materially different behavior;
- ownership cannot be assigned without widening scope;
- required compatibility contradicts a clean migration;
- an overlapping dirty change cannot be preserved safely;
- evidence does not support the proposed physical claim.

Do not stop merely because the task is large; reduce it to the smallest coherent evidence-backed slice.
