# 1Q Module Verification Matrix

Use this checklist proportionally to the touched surfaces. Verify live target names and tool versions before running commands.

## Surface closure

| Surface | Evidence to inspect | Completion signal |
|---|---|---|
| Public API | `include/1q/<module>/`, aggregate header, builders | invalid combinations removed; public types stable and module-owned |
| Validation | config/input validation and unknown enums | invalid values fail closed before mutation |
| Runtime resolution | mapper/resolver/impact classifier | same-value no-op and field-specific state impact tested |
| Pipeline/state | owner, state enum, transition paths | every transition releases or preserves all owned state consistently |
| Snapshot | capture, full pre-mutation validation, restore | corrupt cross-owned state rejects atomically; continuation matches |
| Output layers | raw, result attribution, debug, lifecycle | simulation-only data stays out of raw output |
| Replay | `.fbs`, generated headers, codec, comparator, trace session | roundtrip, unknown-value rejection, transition replay, divergence detection |
| Docs | contract, module design, open questions, review drafts | one authority; unresolved items have re-entry gates |
| Consumer/build | examples, batch validation, install manifest, CMake | first-party consumer builds and module scenarios pass |

## Behavioral test matrix

Cover the rows relevant to the change:

| Scenario | Required assertion |
|---|---|
| nominal cycle | expected physical/output effect, not only internal fields |
| boundary cycle | exact same-cycle or next-successful-cycle behavior |
| invalid input | no state/random consumption; prior valid output behavior remains contractual |
| standby/power-off | pending controls and frozen/reset state follow the frozen rule |
| equal runtime patch | succeeds without state or random reset |
| compatible mode patch | preserves locks/resources and retags deterministically |
| incompatible mode/backend patch | releases only incompatible state and reacquires deterministically |
| random seed patch | resets only owned streams |
| coasting/failure | does not consume forbidden measurements; loss occurs on the exact cycle |
| snapshot restore | uninterrupted and restored continuations match |
| trace/replay | config, patches, pending decisions, outputs, and sources show no divergence |
| unknown enum/schema value | decode rejects without partially mutating the destination |
| multi-target order/capacity | deterministic assignment and documented random-order behavior |

## Validation sequence

1. Inspect `tests/cmake/TestTargets.cmake` and the configured build tree for exact targets.
2. Build focused module unit, integration, contract, and replay targets serially for one preset.
3. Run focused new tests directly when fast feedback is useful.
4. Run module CTest labels and batch validation.
5. Run affected public API, install, dependency-isolation, C++11-header, and docs guards.
6. Search for removed fields, enums, aliases, schema names, comments, examples, and review references.
7. Run `git diff --check`.
8. Inspect `git status --short`, `git diff --stat`, and the full diff for unrelated changes.
9. Commit only after the user asks and all blocking checks pass.

Prefer the release preset documented by the repository. Use the Conan-pinned FlatBuffers generator rather than a system `flatc`; verify the current pin before regeneration.

## Acceptance rules

- Treat compile, contract, replay divergence, structured checks, and explicit behavioral assertions as blocking.
- Treat unsupported physical-trend expectations as warning-only only when the repository contract says so.
- Do not hide failures with disabled tests, relaxed thresholds, or skipped replay comparison.
- Distinguish a current limitation from a regression introduced by the change.
- Report exact commands, passed test counts, residual-search result, and remaining open questions.
