---
Status: draft
Date: 2026-09-03
Review-Baseline: `evidence/rf-full-polarization` @ `fba3ea13`
Authority: 过程脚手架记录（非耐久）；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准；
  权威回写完成、合并进 main 前移除本文档。
---

# rf-full-polarization：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

触发来源：用户代码评审意见——`RfLinkBudget.h` 的 `RfPolarization` 枚举"加一个全极化的枚举值"。
- **证据**：[evidence: include/1q/electromagnetics/RfLinkBudget.h]::RfPolarization（kFullPolarization = 5 已按用户指示先行加入，未提交）

背景白话：
1、极化（polarization）＝电磁波里电场振动的方向；天线接收像对插头插座，方向对得上才收得全。
2、现有五个值＝四种固定振动方式（水平/垂直线极化、左/右旋圆极化）＋一个"方向随机乱跳"的非极化。
3、全极化（full polarization）＝同一套收发系统同时保持两条相互垂直的极化通道（如水平＋竖直各一条）；来波只要有固定振动方式，两条通道合起来就能把能量几乎全收回来。
4、全极化 vs 非极化（最易混）：非极化说的是"波没规律"（属性在对方），全极化说的是"我两条通道都收"（属性在自己）。

现状一句话：枚举值已声明但库内校验不认，处于"一用就被拒绝"的半落地状态，本轮裁定补齐还是回退。
- **证据**：[evidence: src/common/electromagnetics/RfLinkBudget.cpp]::IsKnownPolarization（switch 未列新值）

待裁定项总览（每项四问展开见 §1 矩阵）：
1、半落地枚举值的收敛风险是否成立（冻结"必须补齐或回退，不许停在半落地"）。
2、全极化落点：公共 API 枚举（RfPolarization）与活跃仿真链枚举（RfScenePolarization）是两条并行链，加在哪条或两条都加。
3、全极化对四种固定极化的失配损耗语义（合并 0 dB 还是保守 3.01 dB）。
4、全极化对非极化的失配损耗语义（保守 3.01 dB 还是功率守恒 0 dB）。
5、场景层配套打通范围（字符串拼写、加载器、ESR 校验、回放往返）。
6、默认值与存量行为零变更。
7、非目标边界：RIR 极化散射矩阵（另一套"全极化测量"特性）与极化捷变（时变极化）不触碰。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| 1、半落地枚举值构成"声明即拒绝"风险，必须收敛 | kFullPolarization=5 已进头文件，但库校验拒绝它；停在半落地是最差状态 | [evidence: include/1q/electromagnetics/RfLinkBudget.h]::RfPolarization；[evidence: src/common/electromagnetics/RfLinkBudget.cpp]::IsKnownPolarization | 源码检视：IsKnownPolarization 的 switch 未列新值，TryValidateRfEmissionFrame 与 TryEvaluateRfLink 入口先查它，新值必被拒；`ctest -R "unit::common" -C Release`：1/1 Passed（存量测试不受影响） | 矩阵确认风险，本轮补齐语义或回退枚举值，二选一 | 库内存在能正常消费值 5 的其他入口（检视未发现） | pass（风险成立） |
| 2、需求落点：两条并行极化链，全极化必须落在活跃链才真正可用 | RfPolarization 是给集成方的纯函数 API，src/examples 零调用方；RfScenePolarization 才是活跃链（四模块配置、RF v2 计算、回放、场景 JSON 全走它） | [evidence: include/1q/electromagnetics/RfScene.h]::RfScenePolarization；[evidence: src/common/electromagnetics/RfScene.cpp]；[evidence: include/1q/airborne_radar/config/ArHardwareConfig.h]；[evidence: include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h]；[evidence: include/1q/electronic_countermeasure/EcmTypes.h] | 全仓 grep（已执行）：RfPolarization 在 src/examples 零调用方，仅 RfLinkBudget.cpp 自身与单测引用；RfScenePolarization 被 AR/RIR/ESR/ECM 四模块配置、四个回放编解码器、场景脚本、示例加载器消费 | 用户确认落点范围（仅公共 API / 仅活跃链 / 两侧对称） | 用户表明只要休眠 API 上的声明 | narrow（建议两侧对称加值、语义一次冻结；活跃链是最低要求） |
| 3、全极化对固定极化的失配损耗：合并口径 0 dB | 全极化接收侧对 H/V/R/L 任一固定极化发射 → 0 dB（两条正交通道功率合并，任意固定极化投影 cos²θ+sin²θ=1）；单极化侧收全极化发射 → 3.01 dB（只能收到正交两分量之一）；全极化对全极化 → 0 dB；全极化配对不进 cross_polarization_isolation 分支（定义上两条都收，不存在"正交收不到"） | [evidence: src/common/electromagnetics/RfLinkBudget.cpp]::TryPolarizationLoss、::kLinearCircularLossDb；[evidence: src/common/electromagnetics/RfScene.cpp]::TryPolarizationLoss | 源码检视：现有损耗只有三档——同值 0 dB、同族互垂取 cross_polarization_isolation_db（默认 30）、线圆互配或任一侧非极化 3.0103 dB；[evidence: tests/unit/common/common_rf_link_budget_test.cpp] 已钉 3.0103±1e-4 | 用户裁定取 0 dB 合并口径 | 用户选保守口径（对固定极化也 3.01 dB） | pass（推荐 0 dB——双通道合并是全极化存在的物理意义） |
| 4、全极化对非极化的失配损耗：保守 3.01 dB | 保持现有公理"任一侧非极化固定 3.01 dB"不动，全极化对非极化同取 3.01 dB | [evidence: include/1q/electromagnetics/RfScene.h]（注释块明文该配对规则）；[evidence: src/common/electromagnetics/RfLinkBudget.cpp]::TryPolarizationLoss | 推理：物理上双通道功率相加可回收全部非极化功率（0 dB），但对干扰/杂波场景这意味着全极化接收无代价收满干扰——保守建模取单通道一半（3.01 dB）更安全，且改动最小（现有公理零变更） | 用户裁定取 3.01 dB（保守、公理不变） | 用户裁定取 0 dB（功率守恒口径，干扰计算更严） | pass（推荐 3.01 dB 保守） |
| 5、场景层配套打通范围 | RfScenePolarization 若加值，需配套：字符串拼写、示例加载器、ESR 配置校验上限、回放往返；回放字段全是 int 裸转型，值 5 天然往返，无需 schema 变更 | [evidence: examples/common/config_loaders/remote_identification_radar/config_loader_common.h]::RfScenePolarizationFromString；[evidence: src/electronic_surveillance_radar/session/EsrSessionConfigValidation.cpp]（范围钉 0..4）；[evidence: schemas/replay/rir_session_replay.fbs]、[evidence: schemas/replay/airborne_radar_session_replay.fbs]、[evidence: schemas/replay/esr_session_replay.fbs]、[evidence: schemas/replay/ecm_replay.fbs]（极化字段全为 int）；[evidence: src/remote_identification_radar/session/RirReplayFlatbufferCodec.cpp]（双向 static_cast 无范围检查） | 源码检视（已执行）：1、加载器未知字符串静默回退 kHorizontal（既有陷阱模式，拼错不报错）；2、ESR 校验上限钉死 kUnpolarized，加值后必须同步；3、回放裸转型，值 5 可往返但无测试钉住 | 配套范围进冻结契约（拼写建议 "kFullPolarization" 与现有五个拼写同构；回放往返测试转正） | 用户裁定场景层不动（仅公共 API 加值） | pass |
| 6、默认值与存量行为零变更 | 所有默认值不动：AR/RIR scene_polarization 默认 kHorizontal、ESR polarization 与 ECM transmit_polarization 默认 kUnpolarized、RfEmission/RfReceiverSite 默认 kUnpolarized、场景发射侧硬编码 kHorizontal 不动 | [evidence: include/1q/remote_identification_radar/config/RirHardwareConfig.h]；[evidence: include/1q/airborne_radar/config/ArHardwareConfig.h]；[evidence: include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h]；[evidence: include/1q/electronic_countermeasure/EcmTypes.h]；[evidence: examples/scenes/scene_script.cpp]（发射侧硬编码） | 源码检视默认值清单（已执行，与假设一致） | 冻结契约写明"默认值不变更、存量配置行为逐位不变" | 用户要求改默认 | pass |
| 7、非目标：RIR 极化散射矩阵与极化捷变不触碰 | RIR 已有的极化散射矩阵特性（双通道 RCS、S 矩阵、acceptance-S：span/abs_det/depolarization/psi/tau）是独立的"全极化测量"识别特性，与链路预算极化模式无数据流交联；枚举是"名义极化"常量，不承诺极化捷变（随时间切换极化） | [evidence: tests/unit/remote_identification_radar/rir_polarization_acceptance_s_test.cpp]；[evidence: schemas/replay/rir_replay.fbs]::RirPolarizationRcsSample；[evidence: include/1q/electromagnetics/RfLinkBudget.h]（注释"名义极化"） | 源码检视：acceptance-S 由双通道 RCS 样本驱动，输入来自场景极化 RCS 表，不经过 RfScenePolarization 配对逻辑 | 冻结契约列为非目标 | 用户要求联动（如全极化模式改变 S 矩阵行为） | reject（列入非目标） |

## §2 判定汇总与待裁定问题

判定汇总：
1、半落地风险：pass——必须补齐或回退，不许停在"声明即拒绝"。
2、需求落点：narrow——建议 RfPolarization 与 RfScenePolarization 两侧对称加值、语义一次冻结；活跃链（RfScenePolarization）是最低要求。
3、全极化对固定极化：pass（建议 0 dB 合并口径）——全极化接收对任意固定极化 0 dB；单极化收全极化发射 3.01 dB；全对全 0 dB；不进隔离分支。
4、全极化对非极化：pass（建议 3.01 dB 保守）——现有"非极化=一半"公理不动；若选 0 dB 则干扰/杂波功率计算更严，见待裁定问题 3。
5、场景层配套：pass——字符串拼写、加载器、ESR 校验上限、回放往返测试一次配齐；回放 schema 无需变更。
6、默认值零变更：pass。
7、非目标：reject——RIR 极化散射矩阵、极化捷变明确不碰。

需要用户拍板的问题：
1、落点范围：仅公共 API（RfPolarization）/ 仅活跃链（RfScenePolarization）/ 两侧对称（建议）？
2、全极化对固定极化损耗：0 dB 合并（建议）还是 3.01 dB 保守？
3、全极化对非极化损耗：3.01 dB 保守（建议，现有公理不动）还是 0 dB 功率守恒（干扰/杂波口径更严）？
4、场景层字符串拼写取 "kFullPolarization"（与现有五个拼写同构）是否同意？
5、四个模块配置（AR/RIR 收、ESR 收、ECM 发）是否都允许设全极化，还是先限部分模块？

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、爆炸半径与回滚：下游消费方影响、回退难度（无损/破坏性/回滚注意点）。
5、验收门：构建、聚焦测试、契约测试、特征化测试、探针转正（有回归价值的探针转正式测试）。
6、非目标。
-->

## 修订记录

<!-- 编号条目（修订 1、修订 2……）：日期、裁定内容、来源（用户指令/新证据）；不静默改写既有条目。 -->

## §4 运行记录（Stage C 后填写）

<!-- 对照强制回写清单勾项，全部完成才允许拆脚手架（见 SKILL.md 收尾规则）：
1、实现范围：改动文件与接口。
2、验证命令与结果：命令: pass/fail（含转正的探针测试）。
3、权威回写去向：
   1、正向边界：docs/<module>/design.md（boundaries/data-flow/algorithms 按归属选）。
   2、否决记录：docs/<module>/design.md 的"架构裁定与否决记录"专节（体裁见 docs-governance-standard）。
   3、开放议题：docs/common/open_questions.md 登记（编号）。
   4、证据锁：新增/修订规则后附 - **证据**：[evidence: 路径]。
4、残留风险。
5、后续冻结项。
-->
