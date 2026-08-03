---
name: open-questions-doc-standard
description: Use when creating, editing, auditing, or restructuring docs/common/open_questions.md — the 1Q repo registry of unresolved cross-module architecture questions. Trigger for "开放议题"、"open question"、"OQ 条目"、"open_questions.md 重构"、"已收敛条目迁移"、"议题索引表"、registering a Class D open question after a harden-1q-simulation-module review, or deciding whether a settled item must leave open_questions.md for contract.md/design.md. Enforces the frontmatter block, the status index table, the 5-field entry template (现状/后果/待决问题/当前边界/再进入条件), the evidence-at-file-level rule, and the "settled entries are deleted, not archived" lifecycle.
---

# Open Questions Doc Standard

Define and enforce the structure of `docs/common/open_questions.md`. This file is the
repo's only registry of unresolved, non-normative architecture observations — questions found
during investigation that have **not** yet earned a place in `contract.md` (rules) or a module
`design.md` (design). It is the parking lot for `harden-1q-simulation-module` Class D items
("useful but unproven capability") and for cross-module design disagreements awaiting a Stage A
trigger.

The authority that scopes this file is `docs/common/contract.md` §文档结构: `common/open_questions.md`
records "调查中发现但尚未定论的议题，不构成契约约束". This skill defines how such an entry is written,
indexed, and eventually retired.

## When to use

- Creating a new OQ entry after a review, audit, or debugging session surfaces a design disagreement
  or counter-intuitive behavior that is not yet a defect worth fixing.
- Restructuring or auditing `open_questions.md` (the file reads as a changelog, entries use
  inconsistent fields, settled items are still present).
- Registering a `harden-1q-simulation-module` Class D item with evidence, current boundary, and a
  measurable re-entry condition.
- Deciding whether an entry has converged and must be migrated out (to `contract.md` or `design.md`).
- Reviewing a PR that adds or edits an OQ entry — check it against the 5-field template and the
  evidence rule.

## When not to use

- Writing a rule modules must obey → `docs/common/contract.md`.
- Writing module design detail → the owning `docs/<module>/design.md`.
- Writing build/test/coverage practice → `docs/practice/`.
- Logging a settled decision "for the record" → settled decisions live in `contract.md`/`design.md`;
  `open_questions.md` deletes them (see Lifecycle).

## File skeleton

A well-formed `open_questions.md` has exactly four parts, in order:

```markdown
---
Status: active
Authority: 非规定性记录（不构成契约约束）
Lifecycle: 条目有结论后回写 contract.md 或 design.md 并从本文删除；不保留已收敛条目
Last-reviewed: <YYYY-MM-DD>
---

# 跨模块开放议题

<一段定位>登记调查中发现但尚未定论的跨模块架构议题。每条仅记录现状、后果、待决问题、当前边界
与再进入条件，不构成已批准的实现要求或契约规则。已定论条目必须迁出本文——契约规则进
docs/common/contract.md，模块设计进对应 design.md。

- 何时读本文：<评估某项反直觉/非阻断行为是否为已知边界、查某议题的再进入门槛>
- 何时不读本文：<查必须遵守的规则去 contract.md、查模块设计去 design.md>

## 议题索引

| ID | 域 | 主题 | 一句话 | Status |
|---|---|---|---|---|
| COMMON-OQ-x | common | <主题> | <一句话> | open |
| AR-OQ-x | airborne_radar | <主题> | <一句话> | needs-evidence |

## Common 非阻塞边界

### COMMON-OQ-x：<主题>
<5-field entry>

## Airborne Radar 非阻塞边界
...
```

### Frontmatter

Four fields, all required:

- `Status: active` — always `active` for this file.
- `Authority: 非规定性记录（不构成契约约束）` — fixed wording; this file is explicitly non-normative.
- `Lifecycle: 条目有结论后回写 contract.md 或 design.md 并从本文删除；不保留已收敛条目` — the
  load-bearing rule that keeps the file from rotting into a changelog.
- `Last-reviewed: <YYYY-MM-DD>` — update whenever an entry is added, retired, or re-checked.

### Index table

The index table replaces prose status narratives. One row per live entry, columns:

- **ID** — stable identifier (see Naming).
- **域** — `common`, `airborne_radar`, `electronic_surveillance_radar`, `sar`,
  `electro_optical_sensor`, `sbirs_sensor`.
- **主题** — short noun phrase.
- **一句话** — the behavior or disagreement in one clause.
- **Status** — `open` (understood, awaiting a trigger) or `needs-evidence` (the current-behavior
  claim is not yet backed by a live test/file and needs a probe before it can be acted on).

The index is the reader's 3-second answer to "what is currently open". Deleted (converged) entries
do not appear in the index. Empty ID slots from retired entries are not back-filled.

### Grouping

Group entries by domain with a level-2 heading `<Domain> 非阻塞边界`. Use the single phrase
"非阻塞边界" for every group — do not vary it as "构建边界 / 设计边界 / 仿真边界"; the qualifier adds
no information and makes the file read inconsistently.

## Entry template: five fields

Every entry is a level-3 heading `### <ID>：<主题>` followed by exactly five bullets, in this order:

```markdown
### COMMON-OQ-5：`Step()` 在校验失败/关机时静默复用上一帧

- **现状**：<一两句简要描述>。[evidence: tests/unit/electro_optical_sensor/eos_session]
- **后果**：<当前写法或行为造成的实际后果：footgun、误用风险、跨模块不一致、为何新读者会踩坑>
- **待决问题**：<需要决定什么；保持为一个问句或一组备选>
- **当前边界**：<未决前必须遵守的临时约束；以"不得在文档中宣称…"或"保持现有…"开头>
- **再进入条件 (Stage A)**：<什么真实场景触发重新评估——出现第 N 个消费方、跨模块集成要求统一、
  真实 failure mode 出现等；可量化时给量化阈值>
```

Field guidance:

- **现状** — one or two sentences of plain description, then the evidence tag. Describe what the
  code does today, not the history of how it got there. If a sub-point has already converged into
  `contract.md`/`design.md`, do not re-narrate it here; drop a one-line pointer
  (`收敛子点 → contract.md §X`) if a reader needs the bridge.
- **后果** — the reason this entry is worth keeping. Name the concrete harm: a name that misleads,
  a return value callers forget to check, a silent failure, a cross-module inconsistency. If you
  cannot state a concrete consequence, the entry is not yet ready — either find the harm or drop it.
- **待决问题** — the decision, not the implementation. "是否跨模块统一为 X" not "把 X 改成 Y".
- **当前边界** — the temporary rule callers/authors must respect until the question is resolved.
  Phrased as a prohibition ("不得宣称…") or a hold ("保持现有…").
- **再进入条件 (Stage A)** — the objective trigger that reopens the question. Tie it to a real
  event (second consumer hit, integration requirement, measurable failure), not to "when we have
  time". This is the gate `harden-1q-simulation-module` step 8 asks for.

Keep each entry under roughly 15 lines. If an entry grows past that, it is carrying history
(审查结论、迁移记录、与其他 OQ 关联) that belongs in `contract.md`/`design.md` or in git history,
not here.

### List form inside a field

When a field states **more than one** distinct fact, sub-clause, or option, write it as a numbered
list — one point per line — instead of chaining them with `；` or `——` into a single dense paragraph.
High information density packed into one paragraph is the single biggest readability complaint; the
five-field structure does not by itself solve it if each field is a wall of semicolon-separated prose.

Rules:

- A field with one point stays a single line, no numbering.
- A field with two or more points uses a numbered sub-list (`1.` `2.` `3.`), each item one point.
  Prefer this for 现状 (multiple code paths/behaviors), 后果 (multiple harms), 待决问题 (multiple
  options), 当前边界 (multiple prohibitions), and 再进入条件 (multiple triggers).
- Each numbered item is one sentence/one point — do not nest a second dense clause inside an item.
- Do not number a field that has only one point; decorative numbering adds noise.

Example (note the 现状 field uses numbered items because it states three distinct behaviors):

```markdown
### COMMON-OQ-8：周期输入时间/窗口字段无统一契约，违反时静默拒绝

- **现状**：三模块各自为政的周期时间/窗口校验，违反时多表现为"静默不生效"。
  1. AR 编年史校验拒绝 `window_start_time_s < 上一周期窗口结束`。
  2. ESR 要求 RF 帧窗口字段与周期 input 精确相等，空帧也须填。
  3. EOS 拒绝 `dt_sec > 10/frame_rate_hz`，帧率与步长匹配义务全由调用方承担。
  [evidence: src/electronic_surveillance_radar/validation/EsrInputValidation]
- **后果**：调用方违反时整周期在决策消费点之前被静默拒绝，无显式错误可供察觉。
- **待决问题**：是否跨模块统一周期时间/窗口契约。
- **当前边界**：各模块保持现有校验。不得宣称周期时间戳可任意重复。
- **再进入条件 (Stage A)**：出现第二个真实消费方因时间戳或窗口不匹配而静默失败。
```

## Evidence rule

Evidence in `open_questions.md` is looser than in `contract.md`, on purpose:

- Format: `[evidence: <repo-relative file path>]` — **file level only**.
- Do not append line numbers (`file.cpp:123`), test case names (`file::TestCase`), or function
  names. Those belong in `contract.md`, where evidence is a contract anchor. Here evidence is a
  pointer that says "this file is where I saw the behavior", nothing more.
- One tag per claim is enough; do not stack five `[evidence:]` lines to mimic `contract.md` density.

Rationale: an OQ entry records an observation from investigation, not a frozen behavioral contract.
If the evidence needs contract-grade precision (exact test case, exact assertion), the finding has
outgrown `open_questions.md` and should be promoted to `contract.md` or `design.md` — and then
deleted from here.

## Naming

- Cross-module entries: `COMMON-OQ-<n>`. Module-scoped entries: `<MODULE>-OQ-<n>`
  (`AR-OQ-`, `ESR-OQ-`, `SAR-OQ-`, `EOS-OQ-`, `SBIRS-OQ-`).
- Numbers are stable. When an entry is deleted, its number is not reused and not back-filled; gaps
  are expected and preferred over renumbering (renumbering breaks every external reference).
- The prefix distinguishes scope at a glance — keep cross-module (`COMMON-`) and module-scoped
  prefixes distinct.

## Lifecycle: settled entries are deleted, not archived

This is the rule that keeps the file readable. When an entry reaches a conclusion:

1. Confirm the conclusion has a home. A rule modules must obey → `docs/common/contract.md`. A module
   design fact → the owning `docs/<module>/design.md`. Both should already cite the live test as
   `[evidence: ...]` at contract-grade precision.
2. If a sub-point of the entry is not yet captured elsewhere, migrate it first. Do not delete an
   entry whose only record of "why this code looks the way it does" lives in the OQ prose. (The
   `ControlReducer` classifier-authority sub-point of AR-OQ-2 is the cautionary example: it had to
   move to `design.md` §2.8 before AR-OQ-2 could be deleted.)
3. Delete the entry from `open_questions.md` and remove its row from the index table.
4. Do not leave a "已收敛" stub, a "see contract.md" placeholder, or a retired-entries section. Git
   history is the archive; this file is the live set.

If you are tempted to keep a converged entry "for context", that is the signal it belongs upstream
in `contract.md`/`design.md` — migrate it there with full evidence, then delete it here.

## Anti-patterns

- **Narrative status paragraph.** A prose block at the top listing what converged, what was
  rejected, and what remains. Replace it with the index table; the table is the status.
- **Retaining converged entries.** An entry tagged "已收敛 (date)" but still present. Either it is
  settled (delete it) or it has an open sub-point (split the sub-point into its own entry and
  delete the converged parent).
- **Inconsistent entry shape.** Some entries with 收敛决议/迁移记录/审查结论 fields, others without.
  Use exactly the five fields; everything else is history.
- **Contract-grade evidence in an OQ.** Stacking `[evidence: file::TestCase1]`,
  `[evidence: file::TestCase2]` to look authoritative. If it needs that precision, promote it.
- **Varied group names.** "非阻塞构建边界 / 非阻塞设计边界 / 非阻塞仿真边界". Use one phrase.
- **Renumbering to close gaps.** Reusing a retired entry's number. Never.
- **Vague re-entry condition.** "待后续评估" or "未来考虑". Without an objective trigger the entry
  is unactionable and should be dropped.
- **Dense semicolon prose inside a field.** A 现状 or 待决问题 that chains three behaviors/options
  with `；` into one paragraph. Split multi-point fields into a numbered list (one point per line).

## Self-check

After creating or restructuring the file, verify each item:

1. Frontmatter has all four fields (`Status`, `Authority`, `Lifecycle`, `Last-reviewed`) with the
   fixed wording above; `Last-reviewed` is today.
2. Every live entry has exactly one row in the index table, and every index row maps to exactly one
   entry body. No orphans either direction.
3. Every entry body has exactly the five bullets in order (现状/后果/待决问题/当前边界/再进入条件).
   No extra bullets, no missing bullet.
4. Every `[evidence: ...]` tag points to a repo-relative file path with no line number, no `::`,
   no function name.
5. No entry is tagged or titled "已收敛/已拒绝/已冻结" yet still present. (`grep -n '已收敛\|已拒绝\|已冻结'`
   returns nothing.)
6. Every group heading uses the phrase "非阻塞边界".
7. No retired ID has been reused; gaps from deleted entries remain.
8. Each 再进入条件 names an objective trigger (a real consumer, a real failure, an integration
   requirement), not "future evaluation".
9. No field chains multiple distinct points with `；` into one paragraph. Multi-point fields use a
   numbered sub-list (`1.` `2.` …), one point per line; single-point fields stay one line unnumbered.
