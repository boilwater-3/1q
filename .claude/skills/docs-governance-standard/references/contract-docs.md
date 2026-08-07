# Contract docs writing standard — contract.md / session_contract.md

两份契约文档是跨模块规则的设计权威。写规则时对照本文件。

## 文档定位（先选对文件）

- `docs/common/contract.md` — **所有模块**必须遵守的跨模块契约
  （Public API 边界、会话创建非阻断语义、证据优先开发模式门禁等）。
- `docs/common/session_contract.md` — 仅"有 `*Session` 会话模型的传感器模块"
  （AR/ESR/EOS/SAR/SBIRS）的统一会话契约（SessionConfigBuilder、组合所有权、运行期
  配置提交策略、电源单源、三层输出模型、Replay/trace 语义）。`flight_dynamic`
  无会话模型，不适用。
- 模块级文档不得与这两份冲突；模块自己的细则进 `boundaries.md`。

## Frontmatter

`session_contract.md` 用 YAML frontmatter：

```yaml
---
Status: active
Last-reviewed: <YYYY-MM-DD>
Authority: 有 Session 的传感器模块的统一会话契约
Answers: SessionConfigBuilder、Session 组合所有权、运行期配置提交策略、电源单源、三层输出模型、Replay/trace 语义
---
```

`contract.md` 正文首行起直接写 `Status: active` / `Last-reviewed: ...` / `Authority: ...`
（无 YAML 块），并可在头部标注冻结目标（如 `RF-Interference-Architecture: frozen target`）。

## 规则编号

- 顶层规则：`规则 N`（阿拉伯数字，按文件内顺序递增）。
- 子条款：`a/b/c/d` 小写字母（如 `规则 9` 的 `9a/9b/9c` 三写、`规则 13` 的 `13a/13b/13c/13d`、
  `规则 14` 的 `14a/14b/14c`）。
- 新增规则接在现有编号后；既有编号**不可复用**（历史提交/审查按编号引用）。
- 对齐状态段落：`**对齐状态（YYYY-MM）**：…` 或对齐状态表，逐模块列出谁已对齐、
  谁是参考实现、谁是空洞条款（如 SAR 无逐目标门控 → 13b 空洞条款声明）。

## 规则文本写作要点

1. **先写 scope**：谁必须遵守（"有会话的模块必须对齐" / "所有模块"），谁豁免及原因。
2. **字面格式写死**：code 字符串（`"<module>.validation.<snake_case>"`）、字段名
   （`issues`）、枚举值（`kInputValidation`）、数量级（`~6 值`）——审查按字面核对。
3. **每个断言带证据**：`[evidence: ...]` 指向测试或实现文件/路径
   （如 `[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test]`）。
   对齐状态句中带参考实现（如 `SarDiagnosticUtils::WriteAbort` 为三写参考实现）。
4. **反例写"不得"**：明确禁止形态，如"不得以任何形式复活 dirty flag / Profile 枚举"、
   "`reused_previous_output` 概念已废除"。
5. **边界与例外单独成条**：如 13d 适用范围边界（门控排除 ≠ 目标失效）、14c 豁免注记
   （SAR 运行期聚合码与创建时细分 code 不对齐 → boundaries 注明运行期专有聚合码）。
6. **message 稳定性契约**：人读文本不承诺解析稳定性，机器只认 code——这条本身要写进规则。

## 对齐状态表（多模块规则必备）

`session_contract.md` 规则 14 对齐表形态（Stage C 全部对齐后更新）：

```markdown
**对齐状态（YYYY-MM）**：五模块已全部按本规则对齐（SBIRS/AR/ESR/EOS/SAR），
旧符号（平行字段/缓存/查询 API/枚举）零残留；replay codec/schema/trace 同步完整。
```

表格式（多行时）：

```markdown
| 模块 | 状态 | 说明 |
|---|---|---|
| SAR | 已对齐 | 参考实现；13b 为空洞条款，见 `docs/sar/boundaries.md` |
| … | 已对齐 | … |
```

## 收敛回写的固化写法

OQ 收敛后规则进入契约的典型措辞（`a7809026` 形态）：

- 新增条款："控制器 `RunOnce` 为周期输入校验**权威点**（AR 公共入口为 session 特例）；
  issues 直通，禁止校验缓存与查询 API；校验拒绝不附加粗粒度 abort 条目。"
- 修正既有条款措辞（如"非执行周期"→"校验权威层"设置 abort reason）。
- 对齐状态表补新列（如"校验层归属与 issues 流向"）。

## 自检清单

- [ ] 规则落在正确的文档（跨模块 vs 会话模块）
- [ ] 编号连续、子条款 `a/b/c/d`、无复用
- [ ] scope（谁遵守/谁豁免）明确
- [ ] code/字段/枚举字面格式写死，`[evidence: ...]` 存在且有效
- [ ] 禁止形态（"不得"）与例外边界写清
- [ ] 对齐状态表/段更新，参考实现与空洞条款注明
- [ ] 若为收敛回写：`open_questions.md` 条目已删除，提交 body 按文件列明改动
