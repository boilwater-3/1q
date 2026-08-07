# Workflow checklists — 契约收敛 7 阶段

按阶段顺序执行。每个阶段是一个独立提交；docs 阶段不得并入代码阶段。

## Stage 1 — Freeze（契约冻结）

**提交形态**：`docs(common): freeze <contract-name> (<key decision summary>)`
如 `dac83834 docs(common): freeze unified issue-list contract (single list + phase + optional location)`

检查清单：
- [ ] 规则落在正确的文档：跨模块强制 → `contract.md`；仅"有 Session 的传感器模块" → `session_contract.md`
- [ ] 规则有编号（`规则 N`），子条款用 `a/b/c/d`（如 `规则 14` → `14a/14b/14c`）
- [ ] 规则文本先写清"谁必须遵守"（scope framing），如"五传感器模块必须对齐"
- [ ] 字面格式全部写死：code 字符串（`"<module>.validation.<snake_case>"`）、字段名、枚举值
- [ ] 规则引用的既有参考实现带 `[evidence: ...]` 或文件/路径
- [ ] 提交 body 说明规则的**关键决策**（单一列表 + phase + 可选定位 → 已有 code 保持冻结）
- [ ] 本阶段**不触碰 src/**

## Stage 2 — Per-module implementation（逐模块实施）

**提交形态**：每模块一个提交，`refactor(<module>)` / `feat(<module>)` / `fix(<module>)`
如 `bbfa2a74 refactor(airborne_radar): unify cycle issue list (ArIssue + phase + optional location)`

检查清单：
- [ ] 一个提交只动一个模块（+ 必要的 common 共享层可单独成提交）
- [ ] 提交描述含模块名与关键新类型名，如 `(ArIssue + phase + optional location)`
- [ ] 五模块同构改动保持**逐字同构**（字段顺序、命名、默认值一致）——审查阶段会横向对比
- [ ] 删除旧符号：旧字段、旧枚举、旧查询 API 一次删净，不留半迁移状态
- [ ] 每个 `PROJECT_LOG_*` 调用点保留两行中文注释（`// 中译：…` + `// 标识：…`）
- [ ] 全库 grep 验证旧符号零残留（含 docs 中的叙述句）

## Stage 3 — Test lock（测试锁定）

**提交形态**：`test(<module>): 补 phase 断言锁定规则 N 来源标签`
如 `9fe56112 test(sar): 锁定 kExternalInputRejected→kInputValidation phase 映射`

检查清单：
- [ ] 对规则里每个"易回归点"补断言：phase 赋值、code 字符串化、replay 全字段往返
- [ ] replay roundtrip 测试必须断言**全字段**（severity/phase/code/message/location/field），
      不能只断言 size 或单个字段——否则 schema 新增字段丢失时测试仍绿
- [ ] 校验拒绝路径测试补 `phase == kInputValidation && severity == kError` 断言，
      防止拒绝语义静默翻转
- [ ] 参考形态：SAR `sar_input_validation_test` / `sar_three_write_guard_test` 的 phase 断言

## Stage 4 — Stage C writeback（docs 回写）

**提交形态**：`docs(common): mark <rule N> alignment complete across all five modules`
提交 body 必须含 Validation 块（build + ctest 结果），如 `015db4a6`。

检查清单：
- [ ] `session_contract.md` 对齐状态表更新为 all-aligned
- [ ] `[evidence: ...]` 指针刷新（指向新实现/新测试）
- [ ] **Last-reviewed 元数据随修改更新为当天**（contract/session_contract/boundaries 等
      所有被本次改动触碰的文档都要刷新，否则元数据与内容脱节）
- [ ] 各模块 `boundaries.md` 诊断/状态相关段落重写为统一模型表述
- [ ] 检查各模块 `data-flow.md` 装配点/数据流描述是否过时（本分支发现 ESR/EOS 装配点
      移入 controller 后 data-flow 未同步，`a7809026` 补回写）
- [ ] 逐模块核对：docs 表述与代码行为逐条一致（"validation code 逐字一致"这类条款
      需在 boundaries 明确达成或明确豁免）

## Stage 5 — Open-question register / converge（OQ 登记与收敛）

**登记**：`docs(common): add COMMON-OQ-N for <divergence summary>`
条目格式见 `open-questions-doc-standard`（现状/后果/待决问题/当前边界/再进入条件）。

**收敛**：`docs: COMMON-OQ-N 收敛回写——<what got settled>`
如 `a7809026 docs: COMMON-OQ-9 收敛回写——规则 14 固化校验层归属与 issues 流向`

收敛检查清单：
- [ ] 结论固化进 `session_contract.md` / `contract.md`（成为强制条款），措辞升级为
      权威表述（如"控制器 RunOnce 为校验权威点"）
- [ ] 对齐状态表补新列（如"校验层归属与 issues 流向"）
- [ ] 各模块 boundaries/data-flow 同步更新（含删除过时表述，如 SBIRS 'reuse latest output'）
- [ ] **从 `open_questions.md` 删除该条目**（已收敛条目不保留，见 open-questions-doc-standard）
- [ ] 提交 body 列明每个文档文件改了什么（本分支按文件分条列出）

## Stage 6 — Review doc（审查文档落库）

**提交形态**：`docs(<module>): 审查文档落库` 或 `docs: <topic> 审查落库`
参考：`2a0bd3da docs(sar): 运行期 invalid_config 聚合码契约 14c 文档豁免；审查文档落库`
（审查文档可与模块豁免回写同提交）。

检查清单：
- [ ] 文件名：`docs/review/<topic>_review_<YYYY-MM-DD>.md`（topic 为 kebab-case）
- [ ] frontmatter 四要素：`Status` / `Date` / `Review-Baseline`（分支 @ commit，含
      commits/文件数/增减行统计）/ `Authority`（非规范性声明，不得替代契约）
- [ ] 结构：§0 定位与结论 → §1 判定方法 → §2 发现 → §3 修复优先级 → §4 结论
- [ ] 发现分类：DES（设计）/ BND（边界清理）/ Q（质量与正确性）/ RED（多余与重复）
- [ ] 每条发现：编号（DES-1…）+ 严重度（高/中/低/建议）+ 发现描述 + **证据列**
      （`file:line` 精确引用，可多条）
- [ ] 结论含"机器验证"行（build + ctest 结果）
- [ ] 发现格式详见 `references/review-doc.md`

## Stage 7 — Fix writeback（修复状态回写）

**提交形态**：`docs(common): 审查文档补修复状态与 <extra findings> 注记`
如 `408f523a`。

检查清单：
- [ ] 审查文档顶部追加**后续状态 block**（引用块）：逐条 `编号 → commit hash`
- [ ] 记录**实施中额外发现**（不在原报告列表、修复过程中新暴露的缺陷）及对应修复 commit
- [ ] 记录**暂缓项**（已确认不纳入，如 DES-4/DES-5/RED-1）并说明原因
- [ ] 后续状态 block 注明"全量测试通过"

## 验证（所有阶段通用）

```bash
cmake --build --preset llvm-ninja-release-local
ctest --preset llvm-ninja-release-local --output-on-failure -j 4
git grep -n "<retired-symbol>" -- 'src' 'include' 'tests' 'docs'  # 应零输出
```
