---
Status: draft
Date: 2026-08-07
Review-Baseline: `feature/docs-20260807-093902` @ `a7809026`（25 commits，193 文件，+3179/−2394）
Authority: 非规范性审查记录；不得替代 `docs/common/contract.md`、
`docs/common/session_contract.md` 及各模块 `docs/<module>/design.md`。
若本文与库实现冲突，以库为准。
---

# 规则 14 统一问题列表迁移审查：五模块收敛与 COMMON-OQ-9 闭环

## 0. 定位与结论

本分支实现规则 14（统一问题列表模型：单一 `issues` 列表 + `phase` 来源标签 + 可选定位 +
`<module>.validation.<snake_case>` code 前缀）的完整迁移：契约冻结 → cycle 域五模块落地 →
config 域扩展 → 校验层归属收敛（COMMON-OQ-9 登记→实施→回写关闭），并含 DebugView 三落盘
参考模式与 consumer 样例对齐。

结论：**迁移质量高，核心契约五模块严格落地，无阻断性缺陷，可以合并**。发现 **1 项高优先级
一致性缺陷（AR 校验拒绝三写缺写三）、4 项中优先级清理/测试缺口、若干低优先级项**，全部为
报告项；建议合入前补高优先级一项，其余随后续提交消化。

机器验证：release 增量构建通过；全量 `ctest --preset llvm-ninja-release-local` 通过
（exit 0）。

## 1. 判定方法

- 五个并行只读审查代理（AR/ESR/EOS/SAR/SBIRS），逐提交核对 `git diff main...HEAD`，
  并以全库 grep 验证旧符号残留；
- 本人横切面审查：契约基线（`session_contract.md` 规则 14、COMMON-OQ-9 回写）、common 层
  （`RuntimeCycleExecutor` 清理）、五模块 Issue 结构横向对比、三写链路逐分支核对；
- 构建与测试：`cmake --build --preset llvm-ninja-release-local` + 全量 ctest 通过。

发现分类：DES（设计问题）/ BND（边界清理）/ Q（质量与正确性）/ RED（多余与重复字段）。
严重度：高 / 中 / 低 / 建议。

## 2. 发现

### 2.1 DES：设计问题

| 编号 | 严重度 | 发现 | 证据 |
|---|---|---|---|
| DES-1 | **高** | **AR 校验拒绝路径缺失三写之"写三"（日志），五模块唯一例外**。契约规则 9 三写"缺一不可"；a7809026 明确"拒绝路径显式补齐 abort_reason 与日志"。其他四模块校验拒绝均有 `PROJECT_LOG_WARN("...validation rejected...")`（ESR `EsrController.cpp:105`、SAR `SarController.cpp:71`、SBIRS `SbirsController.cpp:32`、EOS `EosController.cpp:138`）；AR 两条拒绝路径——公共路径 `BuildValidationErrorResult` 与运行期 `ArController::RunOnce`——只有写一（abort_reason）+写二（issues），无任何日志。 | `src/airborne_radar/session/ArSession.cpp:159-171`；`src/airborne_radar/runtime/ArController.cpp:474-478` |
| DES-2 | 中 | **ESR 死枚举 `kRuntimeStateRestoreRejected` 零生成点，清理不彻底**。0d0dd01c 删除 session 层快照回滚分支后，该值无任何赋值点（`EsrController.cpp` 5 个 abort 赋值点均不含），仅剩枚举定义、switch 接收端与 replay 边界校验。新 trace 永不产生但 replay 仍接受——replay 语义与代码语义脱节。`kOutputContractViolation` 同为死值（预存），且 ESR 未被 `check_cross_domain_naming.cmake` 死值守卫覆盖。 | `include/1q/electronic_surveillance_radar/session/EsrOutputTypes.h:84-85`；`src/electronic_surveillance_radar/runtime/EsrController.cpp:59`；`tests/contract/check_cross_domain_naming.cmake:264` |
| DES-3 | 中 | **ESR/EOS 保留零生产调用方的查询 getter**。装配收敛到 `AssembleResult` 后 `latest_result` 已含全部信息，`GetLatestIssues`（ESR/EOS）、`GetLatestCycleStatus`/`GetLastInterceptCycleAbortReason`（ESR）仅被测试使用（`esr_controller_runtime_state_test.cpp`），与"删除校验缓存与查询 API"的收敛方向相抵。AR 的 `GetLatestIssues` 有生产调用（`ArSession.cpp:380`），可保留。 | `src/electro_optical_sensor/runtime/EosController.cpp:203`；`src/electronic_surveillance_radar/runtime/EsrController.h:88-107` |
| DES-4 | 低 | **EOS/ESR status 派生手法不一致**。ESR 缓存 `last_cycle_status`（RunOnce 各出口显式赋值 + 快照字段）；EOS 从 `last_abort_reason` 经 `DeriveCycleStatus` 推导（无状态字段）。powered-off 行为对齐，但"缓存 vs 推导"手法分裂，快照结构亦不同（ESR 含 status、EOS 不含），两边注释互称对齐而实现不同，构成阅读陷阱。 | `src/electronic_surveillance_radar/runtime/EsrController.cpp:36,100`；`src/electro_optical_sensor/runtime/EosController.cpp:85-88` |
| DES-5 | 低 | **`BuildCycleResult(input)` 未使用参数系统性存在**。SAR/ESR/EOS 三模块 `(void)input` 仅签名一致性（SAR 参考实现带头），历史 API 形状，属多余参数。 | `src/sar/runtime/SarController.cpp:126`；`src/electronic_surveillance_radar/runtime/EsrController.h:80`；`src/electro_optical_sensor/runtime/EosController.h:82` |
| DES-6 | 低 | **SAR 运行期聚合码 `invalid_config` 与创建时细分 code 不对齐**。`AreSarHardwareAndMissionFieldsValid` 失败聚合为单一 code，创建时路径产细分 code（`carrier_frequency_not_positive` 等），契约 14c"同条件 code 逐字一致"字面未达成（创建时已校验过，风险低）。 | `src/sar/session/SarRuntimeConfigValidation.cpp:64`；`src/sar/session/SarSessionConfigBuilder.cpp:30-85` |
| DES-7 | 低 | **死分支 `kValidationRejected` 三模块同构残留**。外层 if 已排除校验拒绝，switch 内 case 永不可达（AR 还生成不可达的 `ar.input_validation` code），与"校验拒绝不再走 RecordAbort"注释并存形成语义噪音。 | AR `ArDiagnosticUtils.cpp:38-40` + `ArSession.cpp:181`；ESR `EsrController.cpp:47-49`；EOS `EosController.cpp:73-75` |

### 2.2 BND：边界清理

**总体结论：旧符号清理彻底、零残留（五代理 + 本人 grep 交叉确认）。**

| 编号 | 严重度 | 发现 | 证据 |
|---|---|---|---|
| BND-1 | 低 | EOS `EosController.cpp` 未使用 include `common/runtime/RuntimeCycleExecutor.h`（预存，本分支收敛清理后更显眼）。 | `src/electro_optical_sensor/runtime/EosController.cpp:8` |
| BND-2 | 建议 | ESR `RuntimeCycleState::next_batch_id{1U}` 与 `EsrControllerRuntimeState::next_batch_id{0U}` 初值不一致（预存，Restore 依赖显式赋值，无错，阅读陷阱）。 | `src/common/runtime/RuntimeCycleExecutor.h`；`src/electronic_surveillance_radar/runtime/EsrController.h:31` |

已核查无问题（零残留）：
- `validation_issues` 平行字段、`*ValidationIssue`/`*ValidationIssueList`/`*DiagnosticIssue` 类型、
  `last_validation_issues` 缓存、`GetLastValidationIssues` 查询 API、`ValidationCode`/
  `ConfigValidationCode` 枚举——五模块全库 grep 零残留（docs 中仅"已删除"叙述句）。
- `RuntimeCycleExecutor` 死代码（`RuntimeValidationResult`/`NoValidationIssues`/
  `MakePassValidationResult`/`AdvanceBatchId`）删除彻底，单参数化后 AR/ESR 侧对齐。
- AR 校验缓存访问器与快照字段删除彻底（368f5ec4）；拒绝原因丢失修复完整（2258cc12）——
  所有拒绝分支（公共/运行期/发射后）均直通明细，abort_reason 不写死替换。
- consumer 样例（91beb398）修复正确；DebugView 抽取（3f635cee）`WriteIssuesJson` 输出字节
  不变声明属实；batch_validation 的 `has_validation_error` 为消费端派生列，合规。

### 2.3 Q：质量与正确性

| 编号 | 严重度 | 发现 | 证据 |
|---|---|---|---|
| Q-1 | 中 | **replay roundtrip 测试空洞（AR/ESR）**。AR：构造了含 phase/severity/code/location/field 的 issue 但只断言 field 与 size，与文件头"新增字段未同步立即被发现"承诺不符；ESR：无非空 issues 往返用例。codec 实现本身正确（schema/编解码已同步全字段，SAR/SBIRS 有 fail-closed 范围校验），是测试覆盖问题。EOS/SAR/SBIRS 有真实字段断言（含 SBIRS 哨兵值 `size_t::max ↔ int64(-1)` 往返）。 | `tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp:344-345`；`tests/replay/electronic_surveillance_radar/esr_replay_codec_roundtrip_test.cpp:50-61` |
| Q-2 | 中 | **phase 断言缺口为跨模块系统性弱点**。`HasValidationError` 按 `phase==kInputValidation && severity==kError` 判定，但多数模块校验测试只查 code 字符串：EOS（input 测试零 phase 断言）、SBIRS（`ContainsCode` 只比 code）、AR（rf/config builder 测试无 phase）、SAR（`kExternalInputRejected→kInputValidation` 映射无任何测试锁定——最易回归点）。若误改 phase 赋值，拒绝语义静默翻转而测试仍绿。SAR 的 `sar_input_validation_test`/`sar_three_write_guard_test` 为唯一有 phase 断言的参考形态。 | EOS `tests/unit/electro_optical_sensor/eos_input_validation_test.cpp`；SBIRS `tests/unit/sbirs_sensor/sbirs_session_config_builder_test.cpp:12-19`；SAR `src/sar/session/SarDiagnosticUtils.cpp:50-58` |

已核查无问题：
- 三写路径全部对齐（除 DES-1 AR 校验拒绝缺日志外）；校验 code 字符串化无漏映射
  （AR 19/19、ESR 6+13 逐字对应，SAR/SBIRS 字符串直出无映射表）；
- config 域与周期校验无重复执行（各模块独立入口；SAR `sample_window_too_small_for_pulse`
  双路径产出且字面一致，符合 14c）；
- replay codec/schema/trace 适配器对 phase/location/field 序列化同步完整；
- 构建通过 + 全量测试通过。

### 2.4 RED：多余与重复字段

**总体结论：Issue 结构内部无冗余、无可推导缓存字段残留、无跨模块字段漂移。**

| 编号 | 严重度 | 发现 | 证据 |
|---|---|---|---|
| RED-1 | 低 | **`HasValidationError` 五处手写同逻辑实现**（AR/ESR/EOS/SAR/SBIRS 各一，`phase==kInputValidation && severity==kError` 循环）。类型差异下可接受（模块自治），但共享工具 `HasSeverity`（`src/common/validation/ValidationUtils.h:142`）实际仅 AR 测试使用，处于半死状态——新模块建议直接复用或统一到共享谓词。 | `src/airborne_radar/session/ArInputValidation.cpp:189` 等五处 |

已核查无问题：
- 五模块 `*Issue` 六字段（severity/phase/code/message/location/field）**逐字同构**，顺序、命名、
  默认值（severity=kInfo, phase=kExecution）完全一致，无漂移；
- phase/severity 正交（来源标签 vs 严重级别）、code/message 正交（机器键 vs 人读文本）、
  location/field 正交（定位域+实体索引 vs 字段路径）——均不可互推，非重复字段；
- `*CycleResult`/`*IssueList`/控制器快照均无可推导 error 布尔缓存（`has_error`/
  `has_validation_error` 全删，`executed_this_cycle` 为契约允许的便捷访问器）。

## 3. 建议修复优先级（报告项，未实施）

| 优先级 | 编号 | 问题 | 修复建议 |
|---|---|---|---|
| 高 | DES-1 | AR 校验拒绝三写缺写三 | 公共路径与 `ArController::RunOnce` 校验拒绝分支补 `PROJECT_LOG_WARN`（对齐其余四模块文案形态） |
| 中 | DES-2 | ESR 死枚举 `kRuntimeStateRestoreRejected` | 删除枚举值/switch case，或补注释声明"仅 replay 兼容保留"；评估 `kOutputContractViolation` 与死值守卫覆盖 |
| 中 | DES-3 | ESR/EOS 零生产调用 getter | 删除 `GetLatestIssues`/`GetLatestCycleStatus`/`GetLastInterceptCycleAbortReason`，测试改经 `BuildCycleResult` 断言 |
| 中 | Q-1 | AR/ESR replay roundtrip 空洞 | 补非空 issues 用例并断言 phase/severity/code/message/location/field 全字段往返 |
| 中 | Q-2 | phase 断言缺口 | 各模块 input/config 校验测试补 `phase==kInputValidation` 断言；SAR 补 `kExternalInputRejected→kInputValidation` 锁定 |
| 低 | DES-7 | 死 case `kValidationRejected` | AR/ESR/EOS 三处删除不可达分支 |
| 低 | DES-6 | SAR `invalid_config` 聚合码 | boundaries.md 注明为运行期专有聚合码，或复用细分 code |
| 低 | DES-4/5, BND-1/2, RED-1 | 手法统一与杂项 | 后续清理提交消化 |

## 4. 结论

规则 14 迁移在五模块严格落地：单一 `issues` 列表、`phase` 来源标签、可选定位、`<module>.validation.*`
code 前缀、无可推导缓存字段，均与契约逐条对齐；旧符号零残留；replay codec/schema/trace 同步完整；
构建与全量测试通过。COMMON-OQ-9 收敛（控制器 RunOnce 校验权威点 + issues 直通，AR 公共入口 session
特例）已在契约固化，代码与文档一致。无阻断性缺陷，可合并；建议合入前补 DES-1（AR 校验拒绝日志），
其余为中/低清理项。
