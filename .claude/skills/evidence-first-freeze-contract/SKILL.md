---
name: evidence-first-freeze-contract
description: Use when Codex is asked to plan, review, freeze, or implement high-risk algorithm or architecture changes where evidence should precede implementation. Trigger for SAR/ESR/EOS/radar/flight-dynamic modules, "冻结项", "证据矩阵", "先证明再实现", "证据矩阵交付", "Stage A 文档", "evidence 分支", gen_stage_a_doc, design gates, risky refactors, or cases where rejecting implementation is an acceptable outcome.
---

# Evidence-First Freeze Contract

Use this skill to prevent "implement first, justify later" behavior in high-risk technical work. The core rule is: Stage A builds an evidence matrix; Stage B implementation is allowed only if the evidence proves the need and scope. Evidence rejection is a valid final result.

The flow has a fixed shape: Stage A is delivered as a dated repo document initialized by `scripts/gen_stage_a_doc.py` and committed on a new `evidence/<topic>` branch; after the user closes the discussion, implementation runs on a `feat/<topic>` branch forked from it; Stage C writes durable conclusions into the authority docs and only a run record back into the matrix document.

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

1. Stage A: prove, reject, narrow, or defer the need; deliver the matrix as a script-initialized document on a new `evidence/<topic>` branch.
2. Discussion: the user rules on the proposed decisions; rulings land in the document as numbered revision entries.
3. Contract Freeze: append the frozen contract to the same document before any production-code edit.
4. Stage B: fork `feat/<topic>` from `evidence/<topic>` and implement only the frozen scope.
5. Stage C: verify, write durable conclusions into the authority docs, append the run record to the matrix document.

## Stage A: Evidence Matrix

Whenever this workflow applies, the matrix is delivered as a repo document — never chat-only. The chat reply carries only the decision summary plus a link to the file. Chat-only output is acceptable only for items excluded from this workflow ("Do not use this workflow for").

### Deliverable form

1. Initialize the document with the script; never hand-write the skeleton — the script is the single source of the format, which is what prevents drift:
   `python scripts/gen_stage_a_doc.py <topic>`
2. Path: `docs/review/<topic>_stage_a_<YYYY-MM-DD>.md`, `<topic>` kebab-case. The script fills `Date` and `Review-Baseline` (branch @ HEAD) and refuses to overwrite an existing file.
3. One file grows by stage: `Status` moves draft → frozen (contract appended) → final (run record appended).
4. All-reject and all-defer matrices are also committed — rejection is a valid deliverable.

### Writing rules (whole document, enforced in every section)

1. Evidence is always one line: `- **证据**：[evidence: <path>]`; a `::symbol` suffix is allowed. Line numbers are forbidden — they go stale and cannot be maintained.
2. Keep every explanation brief, one point per line. When several points appear, number them (1、2、3), one per line; large prose blocks are forbidden.
3. Bare references such as "见契约规则 N" are forbidden. State the rule's content directly, then lock its source in evidence form (e.g. `- **证据**：[evidence: docs/common/contract.md]`).
4. Write plain Chinese for non-professional developers; the first occurrence of an unavoidable term gets a one-sentence plain explanation.
5. Probes must have actually been run, with results stated; anything not directly verifiable is labeled `推理：`.

### Matrix columns

The script-generated table headers are the plain-Chinese form below (English shown for column meaning):

| 待裁定项 (Freeze item) | 假设 (Hypothesis) | 证据来源 (Evidence source) | 探针/测试 (Probe/Test) | 通过条件 (Pass criterion) | 否定条件 (Rejection criterion) | 建议判定 (Decision) |
|-------------|------------|-----------------|------------|----------------|--------------------|----------|

Guidance:

- `Freeze item`: the decision under review — a need/risk question ("is this requirement real"), not an implementation idea or a feasibility question.
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

### Delivery constraints

When the matrix is delivered (before the user closes the discussion):

1. §3 冻结契约 and §4 运行记录 stay as script placeholders — no frozen contract, no field-level implementation specs, no Stage C content.
2. Matrix decisions are proposals (建议判定); the user's rulings turn them into numbered revision entries, never silent rewrites.
3. A freeze item must be a need/risk decision; an implementation-feasibility question is not a valid freeze item.

## Evidence Quality

Prefer direct evidence in this order:

1. Existing failing behavior, trace divergence, reproducible test, or concrete acceptance requirement.
2. Source-level contract mismatch, schema/API inconsistency, or documented physical/system constraint.
3. Focused experiment or small characterization test.
4. Reasoned inference from code structure, explicitly labeled as inference.

Avoid treating broad intuition, aesthetic preference, or "future flexibility" as sufficient evidence for risky implementation.

Mark inference explicitly, using the document's `推理：` prefix. For example: "推理：this helper is a composition-root leak because the only caller is session assembly and it pulls pipeline internals into config validation."

## Stage A Outcomes

Stage A may end in any of these outcomes:

- `pass`: enter Stage B with a narrow implementation boundary.
- `reject`: do not implement; record why the requirement was not proven.
- `narrow`: implement only the proven subset.
- `defer`: identify missing evidence and the next probe needed.

When rejecting, be explicit and factual. A rejected implementation is not failure; it preserves engineering quality.

## Branch and Commit Flow

The flow produces two clearly separated commit steps, each on its own branch.

1. Evidence branch — created when the Stage A matrix is delivered.
   - Branch: `evidence/<topic>`.
   - Commits: the script-initialized matrix document, discussion revisions, and the frozen contract (§3). Example: `docs(review): stage-a evidence matrix for <topic>`.
   - Production code never lands here. If Stage A ends in all `reject`/`defer`, this branch with the matrix commit is the final deliverable and no implementation branch is created.
2. Implementation branch — created only after the user closes the discussion and the contract is frozen.
   - Branch: `feat/<topic>`, forked from `evidence/<topic>` so merged history reads: matrix commit(s) → implementation commit(s).
   - All Stage B/C code, test, and authority-doc writeback commits land here, following the `docs-governance-standard` commit shapes (one module per commit; docs commits separate from code commits).

Rules:

1. Never mix matrix-document edits and production-code edits in one commit.
2. Implementation must not re-litigate the matrix: if implementation disproves the contract, stop, return to Stage A, and record the new evidence as a revision on the evidence branch.

## Contract Freeze

Only create an implementation contract when Stage A has at least one `pass` or `narrow` decision. The contract must be written before production-code edits. Append it as §3 of the stage document (the skeleton reserves the section) and commit it on the evidence branch; record the user's closing ruling as a revision entry.

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

Before editing code, fork the implementation branch `feat/<topic>` from `evidence/<topic>` (only after the user closes the discussion), then state:

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

## Stage C: Verification and Authority Writeback

Stage C closes with two writebacks that have different destinations. The matrix document is a working log, not the destination of record.

### Authority writeback (destination of record)

Durable conclusions land in the authority docs that own them, inside the same change:

1. Cross-module rules, session semantics, issue codes → `docs/common/contract.md`, `docs/common/session_contract.md`, `docs/common/issue_codes.md`; open questions registered or converged in `docs/common/open_questions.md`.
2. Module boundary, behavior, data flow, algorithms → the module's `docs/<module>/` set (`design.md` / `boundaries.md` / `data-flow.md` / `algorithms.md`).
3. Acceptance/test-gate conclusions → the document that owns the gate.

Test for durability: if a future reader who was not part of this task needs the conclusion, it belongs in an authority doc. Follow the `docs-governance-standard` Stage-4 writeback commit shape; docs and code stay synchronized per Stage B rules.

### Run record (appended to the matrix document)

The matrix document only records what this run did. Append to its §4, following the document's writing rules:

```markdown
## §4 运行记录

1、实现范围：...
2、验证命令与结果：`command`: pass/fail
3、权威回写去向：哪个结论写进了哪个文件
4、残留风险：...
5、后续冻结项：...
```

If tests fail, do not weaken thresholds or widen skips unless the evidence matrix proves the original acceptance criterion was wrong.

## Recommended Output Shape

For planning/review-only tasks (chat reply):

1. Decision summary (§2 of the document) plus a link to `docs/review/<topic>_stage_a_<date>.md` on branch `evidence/<topic>`.
2. Decision per freeze item.
3. Frozen contract or Stage B scope only for passed/narrowed items.
4. Rejected/deferred items with next evidence needed.

For implementation tasks (on branch `feat/<topic>`):

1. Brief Stage A result with a link to the matrix document.
2. Frozen contract summary.
3. Stage B changes.
4. Verification commands and results.
5. Authority-writeback summary (which conclusion went to which document).
6. Remaining freeze items.
